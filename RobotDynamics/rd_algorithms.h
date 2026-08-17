/**
 * @file rd_algorithms.h
 * @brief Robot Dynamics Algorithms (High-Performance Cached Version)
 * * Merges functionality from Kinematics, Dynamics, Jacobian, and Acceleration.
 * * All algorithms rely on rd_state_t for maximum efficiency.
 */

#ifndef RD_ALGORITHMS_H
#define RD_ALGORITHMS_H

#include "rd_config.h"
#include "rd_chain.h"
#include "rd_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 1. Independent Kinematics (No State Cache Required)
 * ============================================================================ */

/**
 * @brief Compute Forward Kinematics for a specific frame (Standalone)
 * Useful for debugging or computing target poses without a full update.
 */
rd_status_t rd_fk_frame(const rd_chain_t* chain,
                        const rd_real_t* q_base,
                        const rd_real_t* q_joints,
                        rd_idx_t frame_id,
                        rd_real_t T_out[16]);

/* ============================================================================
 * 2. Cached Dynamics & Kinematics (Requires rd_update_kinematics)
 * ============================================================================ */

/**
 * @brief RNEA: Inverse Dynamics (C*qd + g + M*qdd)
 * Computes torques. Pass qdd=NULL for C*qd+g.
 */
rd_status_t rd_rnea_cached(const rd_chain_t* chain,
                           const rd_state_t* state,
                           const rd_real_t* qdd_base,
                           const rd_real_t* qdd_joints,
                           const rd_real_t* gravity,
                           rd_real_t* tau_out);

/**
 * @brief CRBA: Composite Rigid Body Algorithm (Mass Matrix)
 * Computes M(q).
 */
rd_status_t rd_crba_cached(const rd_chain_t* chain,
                           const rd_state_t* state,
                           rd_real_t* M_out);

/**
 * @brief Spatial Acceleration / J_dot * q_dot
 * Computes spatial acceleration of a specific frame.
 * Pass qdd=0 to compute J_dot*q_dot (Coriolis/Centrifugal acceleration).
 */
rd_status_t rd_spatial_acceleration_cached(const rd_chain_t* chain,
                                           const rd_state_t* state,
                                           const rd_real_t* qdd_base,
                                           const rd_real_t* qdd_joints,
                                           rd_idx_t frame_id,
                                           rd_frame_t ref_frame,
                                           rd_real_t a_out[6]);

/**
 * @brief Geometric Jacobian
 * Uses cached T_world to quickly assemble the Jacobian.
 */
rd_status_t rd_jacobian_cached(const rd_chain_t* chain,
                               const rd_state_t* state,
                               rd_idx_t frame_id,
                               rd_frame_t ref_frame,
                               rd_real_t* J_out);

/**
 * @brief Get Spatial Velocity (Zero Cost)
 * Extracts velocity from cache and transforms to desired frame.
 */
rd_status_t rd_get_spatial_velocity_cached(const rd_chain_t* chain,
                                           const rd_state_t* state,
                                           rd_idx_t frame_id,
                                           rd_frame_t ref_frame,
                                           rd_real_t v_out[6]);

/* ============================================================================
 * 3. Helper Wrappers (Zero Overhead)
 * ============================================================================ */

/* Calculate Gravity Compensation terms g(q) */
rd_status_t rd_gravity_compensation(const rd_chain_t* chain,
                                    const rd_state_t* state,
                                    const rd_real_t* gravity,
                                    rd_real_t* tau_g);

/* Calculate Nonlinear Effects C(q,qd)*qd + g(q) */
rd_status_t rd_nonlinear_terms(const rd_chain_t* chain,
                               const rd_state_t* state,
                               const rd_real_t* gravity,
                               rd_real_t* tau_nle);

/* Calculate Coriolis/Centrifugal terms C(q,qd)*qd */
rd_status_t rd_coriolis(const rd_chain_t* chain,
                        const rd_state_t* state,
                        rd_real_t* tau_c);

#ifdef __cplusplus
}
#endif

#endif /* RD_ALGORITHMS_H */
