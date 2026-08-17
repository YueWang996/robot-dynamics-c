/**
 * @file rd_state.c
 * @brief Robot State Implementation
 */

#include "rd_state.h"
#include "rd_math.h"
#include <string.h>

/* Helper for joint transform (copied/inlined for speed) */
static void state_motion_transform(rd_int_t jtype, const rd_real_t axis[3], 
                                   rd_real_t q, rd_real_t T[16]) {
    if (jtype == RD_JOINT_REVOLUTE) {
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

size_t rd_state_buffer_size(rd_int_t n) {
    /* T_pc(16), Ti(16), T_world(16), v(6), a0(6), S(6) per node */
    /* Total: 16+16+16+6+6+6 = 66 floats per node */
    return n * 66 * sizeof(rd_real_t) + 64; /* +64 for alignment safety */
}

rd_status_t rd_state_init(rd_state_t* state, rd_int_t n, void* buffer, size_t size) {
    if (!state || !buffer) return RD_ERR_NULL_PTR;
    if (size < rd_state_buffer_size(n)) return RD_ERR_INVALID_SIZE;
    
    rd_real_t* buf = (rd_real_t*)buffer;
    size_t offset = 0;
    
    state->n_nodes = n;
    state->T_parent_to_child = buf + offset; offset += n * 16;
    state->Ti = buf + offset;                offset += n * 16;
    state->T_world = buf + offset;           offset += n * 16;
    state->v = buf + offset;                 offset += n * 6;
    state->a0 = buf + offset;                offset += n * 6;
    state->S = buf + offset;                 offset += n * 6;
    
    return RD_OK;
}

/* Base pose from [x,y,z, qw,qx,qy,qz]. Matches rd_fk_frame()'s reading of q_base. */
static void state_base_transform(const rd_real_t* q_base, rd_real_t T[16]) {
    if (!q_base) {
        rd_mat4_identity(T);
        return;
    }
    rd_real_t R[9];
    rd_rot_quat(&q_base[3], R);
    rd_rot_to_mat4(R, q_base, T);
}

rd_status_t rd_update_kinematics_fb(const rd_chain_t* chain,
                                    rd_state_t* state,
                                    const rd_real_t* q_base,
                                    const rd_real_t* qd_base,
                                    const rd_real_t* q_joints,
                                    const rd_real_t* qd_joints) {
    if (!chain || !state) return RD_ERR_NULL_PTR;

    rd_int_t n = chain->n_nodes;

    /* Forward Pass */
    for (rd_int_t ti = 0; ti < n; ++ti) {
        rd_idx_t node = chain->topo_order[ti];
        rd_idx_t parent = chain->parent_list[node];
        rd_idx_t jidx = chain->joint_idx[node];
        rd_int_t jtype = chain->joint_type[node];

        /* 1. Compute Transform T_pc (Parent -> Child) */
        rd_real_t T_motion[16];
        if (jidx >= 0 && q_joints) {
            const rd_real_t* ax = &chain->axes[jidx*3];
            state_motion_transform(jtype, ax, q_joints[jidx], T_motion);
        } else {
            rd_mat4_identity(T_motion);
        }

        rd_real_t Ttmp[16], T_pc[16];
        if (parent == -1 && chain->has_floating_base) {
            /* The root's "joint" is the 6-DOF base pose. Compose it ahead of the
             * link's own offsets, which is the order rd_fk_frame() uses when it
             * seeds its accumulator with the base transform. */
            rd_real_t T_base[16];
            state_base_transform(q_base, T_base);
            rd_mat4_mul(T_base, &chain->T_joint_offset[node*16], Ttmp);
            rd_mat4_mul(Ttmp, &chain->T_link_offset[node*16], T_pc);
        } else {
            rd_mat4_mul(&chain->T_joint_offset[node*16], T_motion, Ttmp);
            rd_mat4_mul(Ttmp, &chain->T_link_offset[node*16], T_pc);
        }

        /* Store T_pc */
        memcpy(&state->T_parent_to_child[node*16], T_pc, 16*sizeof(rd_real_t));
        
        /* Store Ti (Child -> Parent) */
        rd_real_t* Ti = &state->Ti[node*16];
        rd_mat4_inv(T_pc, Ti);
        
        /* 2. Compute T_world (for Jacobian/FK) */
        rd_real_t* Tw = &state->T_world[node*16];
        if (parent == -1) {
            /* T_pc already carries the base pose for a floating base, and is
             * relative to the world for a fixed one. */
            memcpy(Tw, T_pc, 16*sizeof(rd_real_t));
        } else {
             rd_mat4_mul(&state->T_world[parent*16], T_pc, Tw);
        }

        /* 3. Compute Joint Subspace S */
        rd_real_t* S_i = &state->S[node*6];
        rd_int_t is_actuated = (jtype == RD_JOINT_REVOLUTE || jtype == RD_JOINT_PRISMATIC);
        rd_real_t v_joint = 0.0f;
        
        if (is_actuated && jidx >= 0) {
            const rd_real_t* axis = &chain->axes[jidx*3];
            rd_real_t twist[6] = {0.0f};
            if (jtype == RD_JOINT_REVOLUTE) {
                twist[3] = axis[0]; twist[4] = axis[1]; twist[5] = axis[2];
            } else {
                twist[0] = axis[0]; twist[1] = axis[1]; twist[2] = axis[2];
            }
            rd_spatial_transform_motion(&chain->T_link_offset[node*16], twist, S_i);
            
            if (qd_joints) v_joint = qd_joints[jidx];
        } else {
            memset(S_i, 0, 6*sizeof(rd_real_t));
        }

        /* 4. Compute Velocity v_i */
        rd_real_t* v_i = &state->v[node*6];
        if (parent == -1) {
            /* Root velocity. qd_base is already expressed in the root body
             * frame, which is the frame every other v_i lives in. */
            if (chain->has_floating_base && qd_base) {
                memcpy(v_i, qd_base, 6*sizeof(rd_real_t));
            } else {
                memset(v_i, 0, 6*sizeof(rd_real_t));
            }
        } else {
            /* v_i = Ad(Ti) * v_parent + S * qd */
            rd_real_t v_parent_trans[6];
            rd_spatial_transform_motion(Ti, &state->v[parent*6], v_parent_trans);
            for(int k=0; k<6; ++k) v_i[k] = v_parent_trans[k] + S_i[k] * v_joint;
        }
    }
    return RD_OK;
}

rd_status_t rd_update_kinematics(const rd_chain_t* chain,
                                 rd_state_t* state,
                                 const rd_real_t* q_joints,
                                 const rd_real_t* qd_joints) {
    return rd_update_kinematics_fb(chain, state, NULL, NULL, q_joints, qd_joints);
}