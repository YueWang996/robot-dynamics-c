/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_state.c
 * @brief Per-tick state cache implementation.
 */

#include "rd_state.h"
#include "rd_math.h"
#include <string.h>

/* Base pose from [x,y,z, qw,qx,qy,qz]. Matches rd_fk_frame()'s reading. */
static void state_base_transform(const rd_real_t* q_base, rd_real_t T[16]) {
    if (!q_base) {
        rd_mat4_identity(T);
        return;
    }
    rd_real_t R[9];
    rd_rot_quat(&q_base[3], R);
    rd_rot_to_mat4(R, q_base, T);
}

size_t rd_state_buffer_size(rd_int_t n) {
    return (size_t)RD_STATE_BUF_FLOATS(n) * sizeof(rd_real_t);
}

rd_status_t rd_state_init(rd_state_t* state, rd_int_t n,
                          void* buffer, size_t size) {
    if (!state || !buffer) return RD_ERR_NULL_PTR;
    if (n <= 0) return RD_ERR_INVALID_SIZE;
    if (size < rd_state_buffer_size(n)) return RD_ERR_INVALID_SIZE;

    rd_real_t* buf = (rd_real_t*)buffer;
    size_t off = 0;

    state->n_nodes = n;
    /* Nothing is cached yet, and an uninitialised value here that happened to
     * match a chain pointer would silently skip computing every transform. */
    state->cached_for = NULL;

    state->T_world           = buf + off; off += (size_t)n * 16;
    state->T_dyn             = buf + off; off += (size_t)n * 16;
    state->v                 = buf + off; off += (size_t)n * 6;
    state->vj                = buf + off; off += (size_t)n;

    state->inertia           = buf + off; off += (size_t)n * 36;
    state->accel             = buf + off; off += (size_t)n * 6;
    state->force             = buf + off; off += (size_t)n * 6;
    state->cvp               = buf + off; off += (size_t)n * 6;
    state->U                 = buf + off; off += (size_t)n * 6;
    state->D                 = buf + off; off += (size_t)n;
    state->u                 = buf + off; off += (size_t)n;

    return RD_OK;
}

rd_status_t rd_update_kinematics(const rd_chain_t* chain,
                                 rd_state_t* state,
                                 const rd_real_t* q_base,
                                 const rd_real_t* q_joints,
                                 const rd_real_t* qd) {
    if (!chain || !state) return RD_ERR_NULL_PTR;
    if (state->n_nodes < chain->n_nodes) return RD_ERR_INVALID_SIZE;

    const rd_int_t n = chain->n_nodes;
    const rd_int_t base_dof = chain->has_floating_base ? 6 : 0;

    /* Most of what this loop computes does not depend on the configuration at
     * all. The motion subspace S is a function of the joint axis and the link
     * offset, so it is fixed once the chain is built -- for every node, moving
     * or not. And for a node whose joint cannot move, so is T_dyn: two 4x4 SE3 composes and an inverse per tick, arriving
     * at the same numbers forever. That is not a rare case. A robot's fixture
     * links -- feet, sensor mounts, inertial frames -- are all fixed joints,
     * and Go2 has 18 of them out of 30 nodes.
     *
     * So compute them once per (chain, state) pairing. Keyed on the chain
     * rather than latched, so that driving two models through one state buffer
     * recomputes instead of quietly reading the other robot's transforms. A
     * chain mutated after it is built would go stale, but rd_chain_build is a
     * one-time call and nothing in the API invites that. */
    const int cached = (state->cached_for == (const void*)chain);

    /* Cold pass, once per (chain, state) pairing: everything the configuration
     * cannot change: a fixed link's pose in its nearest moving ancestor. After
     * this the per-tick loop below never has to look at a fixed link again. */
    if (!cached) {
        for (rd_int_t ti = 0; ti < n; ++ti) {
            rd_idx_t node = chain->topo_order[ti];
            if (rd_chain_node_is_dynamic(chain, node)) continue;

            /* A fixed link contributes only its joint offset, composed through
             * any fixed links between it and its moving ancestor. */
            rd_idx_t parent = chain->parent_list[node];
            rd_real_t* Td = &state->T_dyn[node*16];
            if (rd_chain_node_is_dynamic(chain, parent)) {
                memcpy(Td, &chain->T_joint_offset[node*16],
                       16 * sizeof(rd_real_t));
            } else {
                rd_mat4_mul_se3(&state->T_dyn[parent*16],
                                &chain->T_joint_offset[node*16], Td);
            }
        }
    }

    /* Per tick: only the nodes that can move. */
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node   = chain->dyn_order[di];
        rd_idx_t parent = chain->parent_list[node];
        rd_idx_t danc   = chain->dyn_parent[node];
        rd_idx_t jidx   = chain->joint_idx[node];
        rd_int_t jtype  = chain->joint_type[node];

        const int root_fb  = (parent == -1 && chain->has_floating_base);
        const int actuated = (jidx >= 0) && (jtype == RD_JOINT_REVOLUTE ||
                                             jtype == RD_JOINT_PRISMATIC);
        rd_real_t* Td  = &state->T_dyn[node*16];

        /* Built straight into T_dyn when the parent is itself a dynamics node,
         * because then the two are the same transform. That is exactly when
         * the moving ancestor and the parent coincide. */
        const int under_dyn = (danc == parent);

        /* 1. Pose in the nearest moving ancestor */
        {
            rd_real_t T_local[16];
            rd_real_t* T_pc = under_dyn ? Td : T_local;

            if (root_fb) {
                /* The root's "joint" is the 6-DOF base pose, composing ahead of
                 * the link's own offset -- the order rd_fk_frame() uses when it
                 * seeds its accumulator with the base transform. */
                rd_real_t T_base[16];
                state_base_transform(q_base, T_base);
                rd_mat4_mul_se3(T_base, &chain->T_joint_offset[node*16], T_pc);
            } else if (actuated && q_joints) {
                rd_mat4_mul_joint(&chain->T_joint_offset[node*16],
                                  chain->s_axis[node], chain->s_sign[node],
                                  q_joints[jidx], T_pc);
            } else {
                memcpy(T_pc, &chain->T_joint_offset[node*16],
                       16 * sizeof(rd_real_t));
            }
            if (!under_dyn) {
                rd_mat4_mul_se3(&state->T_dyn[parent*16], T_pc, Td);
            }
        }

        rd_real_t v_joint = (actuated && qd) ? qd[base_dof + jidx] : RD_REAL(0.0);
        state->vj[node] = v_joint;

        /* 2. Pose in the world, through the moving ancestor rather than the
         *    immediate parent, so the fixed links between need no visit. */
        rd_real_t* Tw = &state->T_world[node*16];
        if (danc == -1) memcpy(Tw, Td, 16*sizeof(rd_real_t));
        else rd_mat4_mul_se3(&state->T_world[danc*16], Td, Tw);

        /* 3. Spatial velocity, in this link's body frame */
        rd_real_t* v_i = &state->v[node*6];
        if (parent == -1) {
            if (chain->has_floating_base && qd) {
                memcpy(v_i, qd, 6*sizeof(rd_real_t));
            } else {
                memset(v_i, 0, 6*sizeof(rd_real_t));
            }
        } else {
            rd_spatial_transform_motion_inv(Td, &state->v[danc*6], v_i);
            if (actuated) {
                v_i[chain->s_axis[node]] += chain->s_sign[node] * v_joint;
            }
        }
    }

    state->cached_for = (const void*)chain;
    return RD_OK;
}
