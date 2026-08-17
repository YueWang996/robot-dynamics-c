/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_state.c
 * @brief Per-tick state cache implementation.
 */

#include "rd_state.h"
#include "rd_math.h"
#include <string.h>

/* Joint motion transform for a 1-DOF joint. */
static void state_motion_transform(rd_int_t jtype, const rd_real_t axis[3],
                                   rd_real_t q, rd_real_t T[16]) {
    if (jtype == RD_JOINT_REVOLUTE) {
        if (rd_mat4_axis_rotation(axis, q, T)) return;
        rd_real_t R[9];
        rd_rot_axis_angle(axis, q, R);
        rd_real_t t0[3] = {0};
        rd_rot_to_mat4(R, t0, T);
    } else if (jtype == RD_JOINT_PRISMATIC) {
        rd_real_t t[3] = {axis[0]*q, axis[1]*q, axis[2]*q};
        rd_mat4_translate(t, T);
    } else {
        rd_mat4_identity(T);
    }
}

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

    state->T_parent_to_child = buf + off; off += (size_t)n * 16;
    state->Ti                = buf + off; off += (size_t)n * 16;
    state->T_world           = buf + off; off += (size_t)n * 16;
    state->v                 = buf + off; off += (size_t)n * 6;
    state->S                 = buf + off; off += (size_t)n * 6;
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

    for (rd_int_t ti = 0; ti < n; ++ti) {
        rd_idx_t node   = chain->topo_order[ti];
        rd_idx_t parent = chain->parent_list[node];
        rd_idx_t jidx   = chain->joint_idx[node];
        rd_int_t jtype  = chain->joint_type[node];

        /* 1. Transform from the parent link to this one */
        rd_real_t Ttmp[16], T_pc[16];
        if (parent == -1 && chain->has_floating_base) {
            /* The root's "joint" is the 6-DOF base pose. It composes ahead of
             * the link's own offsets, which is the order rd_fk_frame() uses
             * when it seeds its accumulator with the base transform. */
            rd_real_t T_base[16];
            state_base_transform(q_base, T_base);
            rd_mat4_mul_se3(T_base, &chain->T_joint_offset[node*16], Ttmp);
            rd_mat4_mul_se3(Ttmp, &chain->T_link_offset[node*16], T_pc);
        } else {
            rd_real_t T_motion[16];
            if (jidx >= 0 && q_joints) {
                state_motion_transform(jtype, &chain->axes[jidx*3],
                                       q_joints[jidx], T_motion);
            } else {
                rd_mat4_identity(T_motion);
            }
            rd_mat4_mul_se3(&chain->T_joint_offset[node*16], T_motion, Ttmp);
            rd_mat4_mul_se3(Ttmp, &chain->T_link_offset[node*16], T_pc);
        }

        memcpy(&state->T_parent_to_child[node*16], T_pc, 16*sizeof(rd_real_t));

        rd_real_t* Ti = &state->Ti[node*16];
        rd_mat4_inv(T_pc, Ti);

        /* 2. Pose in the world */
        rd_real_t* Tw = &state->T_world[node*16];
        if (parent == -1) {
            /* T_pc already carries the base pose for a floating base, and is
             * relative to the world for a fixed one. */
            memcpy(Tw, T_pc, 16*sizeof(rd_real_t));
        } else {
            rd_mat4_mul_se3(&state->T_world[parent*16], T_pc, Tw);
        }

        /* 3. Joint motion subspace */
        rd_real_t* S_i = &state->S[node*6];
        rd_int_t actuated = (jtype == RD_JOINT_REVOLUTE || jtype == RD_JOINT_PRISMATIC);
        rd_real_t v_joint = RD_REAL(0.0);

        if (actuated && jidx >= 0) {
            const rd_real_t* axis = &chain->axes[jidx*3];
            rd_real_t twist[6] = {0};
            if (jtype == RD_JOINT_REVOLUTE) {
                twist[3] = axis[0]; twist[4] = axis[1]; twist[5] = axis[2];
            } else {
                twist[0] = axis[0]; twist[1] = axis[1]; twist[2] = axis[2];
            }
            rd_spatial_transform_motion(&chain->T_link_offset[node*16], twist, S_i);
            if (qd) v_joint = qd[base_dof + jidx];
        } else {
            memset(S_i, 0, 6*sizeof(rd_real_t));
        }
        state->vj[node] = v_joint;

        /* 4. Spatial velocity, in this link's body frame */
        rd_real_t* v_i = &state->v[node*6];
        if (parent == -1) {
            if (chain->has_floating_base && qd) {
                memcpy(v_i, qd, 6*sizeof(rd_real_t));
            } else {
                memset(v_i, 0, 6*sizeof(rd_real_t));
            }
        } else {
            rd_real_t v_parent[6];
            rd_spatial_transform_motion(Ti, &state->v[parent*6], v_parent);
            for (int k = 0; k < 6; ++k) v_i[k] = v_parent[k] + S_i[k] * v_joint;
        }
    }
    return RD_OK;
}
