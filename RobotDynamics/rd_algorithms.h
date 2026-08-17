/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_algorithms.h
 * @brief Kinematics and dynamics algorithms.
 *
 * Every function here takes (chain, state) and reads the cache that
 * rd_update_kinematics() filled for this tick. None of them allocate: all
 * scratch lives in rd_state_t.
 *
 * Naming and argument order mirror the PyTorch library bard, so the same
 * algorithm is called the same thing on both sides.
 *
 *   bard                                  RobotDynamics
 *   ------------------------------------  ------------------------------------
 *   update_kinematics(model, data, q, qd) rd_update_kinematics(chain, state, ...)
 *   forward_kinematics(model, data, fid)  rd_forward_kinematics(chain, state, fid, T)
 *   jacobian(model, data, fid, ref)       rd_jacobian(chain, state, fid, ref, J)
 *   rnea(model, data, qdd, gravity)       rd_rnea(chain, state, qdd, gravity, tau)
 *   crba(model, data)                     rd_crba(chain, state, M)
 *   aba(model, data, tau, gravity)        rd_aba(chain, state, tau, gravity, qdd)
 *   spatial_acceleration(...)             rd_spatial_acceleration(...)
 *
 * Vectors in velocity space -- qdd, tau, and the columns of J and M -- are
 * packed to length nv, with the base in the first six elements for a floating
 * base. See rd_state.h for the full convention.
 *
 * `gravity` is a 3-vector in WORLD coordinates, or NULL for (0, 0, -9.81).
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
 * Kinematics
 * ============================================================================ */

/**
 * @brief Pose of one frame in the world, without a state cache.
 *
 * Walks only the root-to-frame path, so this is much cheaper than a full
 * rd_update_kinematics() when you need a single frame and nothing else.
 *
 * @param q_base [7] or NULL; @param q_joints [nj] or NULL.
 * @param T_out  [16] column-major 4x4.
 */
rd_status_t rd_fk_frame(const rd_chain_t* chain,
                        const rd_real_t* q_base,
                        const rd_real_t* q_joints,
                        rd_idx_t frame_id,
                        rd_real_t T_out[16]);

/** @brief Pose of one frame in the world, read straight out of the cache. */
rd_status_t rd_forward_kinematics(const rd_chain_t* chain,
                                  const rd_state_t* state,
                                  rd_idx_t frame_id,
                                  rd_real_t T_out[16]);

/** @brief Geometric Jacobian of a frame. @param J_out [6*nv] row-major. */
rd_status_t rd_jacobian(const rd_chain_t* chain,
                        const rd_state_t* state,
                        rd_idx_t frame_id,
                        rd_frame_t ref_frame,
                        rd_real_t* J_out);

/** @brief Spatial velocity of a frame. @param v_out [6]. */
rd_status_t rd_spatial_velocity(const rd_chain_t* chain,
                                const rd_state_t* state,
                                rd_idx_t frame_id,
                                rd_frame_t ref_frame,
                                rd_real_t v_out[6]);

/**
 * @brief Spatial acceleration of a frame.
 * @param qdd [nv] or NULL. Pass NULL to get the velocity-product term alone,
 *            i.e. J_dot * qd.
 */
rd_status_t rd_spatial_acceleration(const rd_chain_t* chain,
                                    const rd_state_t* state,
                                    const rd_real_t* qdd,
                                    rd_idx_t frame_id,
                                    rd_frame_t ref_frame,
                                    rd_real_t a_out[6]);

/* ============================================================================
 * Dynamics
 * ============================================================================ */

/**
 * @brief Inverse dynamics: tau = M(q) qdd + C(q,qd) qd + g(q).
 * @param qdd     [nv] or NULL for zero acceleration.
 * @param tau_out [nv].
 */
rd_status_t rd_rnea(const rd_chain_t* chain,
                    const rd_state_t* state,
                    const rd_real_t* qdd,
                    const rd_real_t* gravity,
                    rd_real_t* tau_out);

/**
 * @brief Forward dynamics: qdd = M(q)^-1 (tau - C(q,qd) qd - g(q)).
 *
 * Articulated Body Algorithm, O(n). For a floating base the first six elements
 * of `tau` are the external wrench on the base in its own body frame -- pass
 * zeros there for a free-flying robot -- and the first six of `qdd_out` are the
 * resulting base acceleration.
 *
 * @param tau     [nv] or NULL for zero torque.
 * @param qdd_out [nv].
 * @return RD_ERR_SINGULAR if the articulated body inertia is not positive
 *         definite, which means the model has a zero-inertia moving link.
 */
rd_status_t rd_aba(const rd_chain_t* chain,
                   const rd_state_t* state,
                   const rd_real_t* tau,
                   const rd_real_t* gravity,
                   rd_real_t* qdd_out);

/** @brief Joint-space mass matrix M(q). @param M_out [nv*nv] row-major. */
rd_status_t rd_crba(const rd_chain_t* chain,
                    const rd_state_t* state,
                    rd_real_t* M_out);

/* ============================================================================
 * Convenience wrappers
 * ============================================================================ */

/**
 * @brief Gravity torques g(q) alone.
 *
 * Runs the RNEA recursion with the cached velocities suppressed, so this is
 * genuinely g(q) and not the full nonlinear term -- you do not need to rebuild
 * the state with qd = 0.
 */
rd_status_t rd_gravity(const rd_chain_t* chain,
                       const rd_state_t* state,
                       const rd_real_t* gravity,
                       rd_real_t* tau_out);

/** @brief Nonlinear effects C(q,qd) qd + g(q). */
rd_status_t rd_nonlinear_terms(const rd_chain_t* chain,
                               const rd_state_t* state,
                               const rd_real_t* gravity,
                               rd_real_t* tau_out);

/** @brief Coriolis and centrifugal terms C(q,qd) qd, with gravity removed. */
rd_status_t rd_coriolis(const rd_chain_t* chain,
                        const rd_state_t* state,
                        rd_real_t* tau_out);

#ifdef __cplusplus
}
#endif

#endif /* RD_ALGORITHMS_H */
