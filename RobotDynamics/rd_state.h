/**
 * @file rd_state.h
 * @brief Robot State Cache to eliminate redundant calculations
 */

#ifndef RD_STATE_H
#define RD_STATE_H

#include "rd_config.h"
#include "rd_chain.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Robot Dynamics State Cache
 * Stores computed transforms and velocities to avoid re-calculation.
 */
typedef struct {
    /* Buffers */
    rd_real_t* T_parent_to_child; /* 16 * n_nodes (Stores T_pc) */
    rd_real_t* Ti;                /* 16 * n_nodes (Stores T_child_to_parent) */
    rd_real_t* T_world;           /* 16 * n_nodes (Optional, for Jacobian/FK) */
    rd_real_t* v;                 /* 6 * n_nodes (Spatial velocities) */
    rd_real_t* a0;                /* 6 * n_nodes (Zero-accel/Coriolis accelerations) */
    rd_real_t* S;                 /* 6 * n_nodes (Joint subspace) */
    
    rd_int_t n_nodes;
} rd_state_t;

/**
 * @brief Initialize state buffer
 * Note: Does not allocate internal arrays if RD_USE_STATIC_ALLOC is 0. 
 * User must malloc or point buffers to valid memory.
 */
rd_status_t rd_state_init(rd_state_t* state, rd_int_t n_nodes, void* buffer, size_t buffer_size);

/**
 * @brief Calculate required buffer size for state
 */
size_t rd_state_buffer_size(rd_int_t n_nodes);

/**
 * @brief Update Kinematics for a floating-base model (Transforms & Velocities)
 *
 * Call this ONCE per control loop. Computes: T_parent_to_child, Ti, T_world, S, v.
 *
 * Conventions
 * -----------
 *  q_base  : 7 elements, [x, y, z, qw, qx, qy, qz]. Position of the root link
 *            in the world frame, then its orientation as a quaternion with the
 *            scalar part FIRST. Pass NULL to place the base at the identity.
 *
 *  qd_base : 6 elements, [vx, vy, vz, wx, wy, wz]. Spatial velocity of the root
 *            link expressed in the ROOT LINK'S OWN BODY FRAME, not in world
 *            coordinates. This matches the free-flyer convention used by
 *            Pinocchio, and it is the frame the base columns of
 *            rd_jacobian_cached() are expressed in. Pass NULL for a base at
 *            rest. To convert a world-frame twist v_w, apply
 *            rd_spatial_transform_motion(state->Ti[root*16], v_w, qd_base).
 *
 * Both are ignored when the chain has no floating base.
 */
rd_status_t rd_update_kinematics_fb(const rd_chain_t* chain,
                                    rd_state_t* state,
                                    const rd_real_t* q_base,
                                    const rd_real_t* qd_base,
                                    const rd_real_t* q_joints,
                                    const rd_real_t* qd_joints);

/**
 * @brief Update Kinematics with the base held at the identity pose, at rest.
 *
 * Equivalent to rd_update_kinematics_fb(chain, state, NULL, NULL, q, qd).
 * This is the right call for fixed-base robots. For a floating-base robot it
 * computes a valid but base-independent state -- use the _fb form instead.
 */
rd_status_t rd_update_kinematics(const rd_chain_t* chain,
                                 rd_state_t* state,
                                 const rd_real_t* q_joints,
                                 const rd_real_t* qd_joints);

#ifdef __cplusplus
}
#endif

#endif /* RD_STATE_H */