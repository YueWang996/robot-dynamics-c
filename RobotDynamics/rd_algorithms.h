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


/* ----------------------------------------------------------------------------
 * External forces
 *
 * `f_ext` is [6*n_nodes] or NULL, one spatial force per *node* -- the same
 * indexing rd_forward_kinematics() uses, not the velocity indexing of tau --
 * ordered [linear, angular] and expressed in that link's own body frame. It is
 * the force the world applies to the link.
 *
 * Indexing by node is what lets a contact sit where it physically does. Feet,
 * fingertips and sensor pads are usually *fixed* links, which rd_chain_build
 * folds into the moving link above them; a force placed on one is carried to
 * that link automatically, so callers do not have to know which links survived
 * the folding.
 *
 * To turn a world-frame contact force into an entry: rotate it by the
 * transpose of the link's world rotation, which rd_forward_kinematics() gives.
 * Links with no force cost six comparisons each, and passing NULL costs
 * nothing at all.
 * ------------------------------------------------------------------------- */

/**
 * @brief Inverse dynamics with external forces:
 *        tau = M qdd + C qd + g - sum_i J_i^T f_ext_i.
 *
 * The subtraction happens inside the O(n) recursion, so contacts cost a few
 * additions rather than a Jacobian and a 6xnv product per contact.
 *
 * @param qdd   [nv] or NULL for zero acceleration.
 * @param f_ext [6*n_nodes] or NULL. See above.
 */
rd_status_t rd_rnea_ext(const rd_chain_t* chain,
                        const rd_state_t* state,
                        const rd_real_t* qdd,
                        const rd_real_t* gravity,
                        const rd_real_t* f_ext,
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
#if RD_ENABLE_ABA
rd_status_t rd_aba(const rd_chain_t* chain,
                   const rd_state_t* state,
                   const rd_real_t* tau,
                   const rd_real_t* gravity,
                   rd_real_t* qdd_out);

/**
 * @brief Forward dynamics with external forces.
 *
 * The same recursion, with each link's external force entering its bias force.
 * @param f_ext [6*n_nodes] or NULL. See the note above rd_rnea_ext().
 */
rd_status_t rd_aba_ext(const rd_chain_t* chain,
                       const rd_state_t* state,
                       const rd_real_t* tau,
                       const rd_real_t* gravity,
                       const rd_real_t* f_ext,
                       rd_real_t* qdd_out);
#endif /* RD_ENABLE_ABA */

/* ============================================================================
 * Linear algebra
 *
 * The factorisation the forward dynamics already uses, exposed because the
 * things people build on a mass matrix -- operational-space inertia, a
 * task-space controller, a constrained solve -- all want it and should not
 * have to bring their own.
 * ========================================================================= */

/**
 * @brief Cholesky factorisation of an n x n symmetric positive definite matrix.
 *
 * A is row-major and is overwritten with L in its lower triangle; the upper
 * triangle is left as it was. Only the lower triangle of the input is read, so
 * a mass matrix from rd_crba() can be passed straight in.
 *
 * @param dinv [n] scratch, filled with 1/L_ii, which rd_cholesky_solve() needs.
 * @return RD_ERR_SINGULAR if A is not positive definite.
 */
rd_status_t rd_cholesky_factor(rd_real_t* A, rd_int_t n, rd_real_t* dinv);

/**
 * @brief Solve A x = b from rd_cholesky_factor()'s output.
 *
 * One factorisation serves any number of right-hand sides, which is the point
 * of having the two apart: J M^-1 J^T costs one factorisation and six solves.
 * x and b may not alias.
 */
rd_status_t rd_cholesky_solve(const rd_real_t* L, const rd_real_t* dinv,
                              const rd_real_t* b, rd_real_t* x, rd_int_t n);

/** @brief Joint-space mass matrix M(q). @param M_out [nv*nv] row-major. */
rd_status_t rd_crba(const rd_chain_t* chain,
                    const rd_state_t* state,
                    rd_real_t* M_out);

/**
 * @brief Which recursion rd_forward_dynamics() should use.
 *
 * Both give the same qdd. Which is faster is a property of the robot, not of
 * the library, so it is the caller's choice rather than a heuristic here:
 *
 *   RD_FD_ABA    Featherstone's articulated-body algorithm. O(n), needs no
 *                workspace beyond rd_state_t, and is the only option if you
 *                cannot spare nv*nv floats. Wins as the model grows.
 *
 *   RD_FD_CRBA   Build M(q) and h(q,qd), then solve M qdd = tau - h by
 *                Cholesky. O(n^3) in the solve, but the mass matrix and the
 *                bias term are cheap recursions and the solve is small, so for
 *                the model sizes a microcontroller runs it is usually the
 *                faster of the two. It also hands you M and h, which an
 *                operational-space controller wants anyway.
 *
 * ABA is also what sizes rd_state_t -- it is the only algorithm with per-node
 * state of its own -- so a build that has settled on RD_FD_CRBA can set
 * RD_ENABLE_ABA=0 and get 25 floats per link back. rd_forward_dynamics() then
 * answers RD_ERR_INVALID_INDEX to RD_FD_ABA.
 *
 * Forward dynamics = update_kinematics + the method. How much the winner wins
 * by, on two Cortex-M4F parts that agree about it -- an STM32L413 at 80 MHz
 * and an STM32G474 at 170 MHz:
 *
 *                                   winner    L413   G474
 *   spine   9 dof, floating base    ABA        -4%    -4%
 *   xarm7   7 dof, fixed base       CRBA      -23%   -23%
 *   go2    18 dof, floating base    ABA       -12%   -11%
 *   g1     35 dof, floating base    ABA       -47%   -46%
 *
 * Two things move the line, in opposite directions. The solve grows as nv^3
 * with nothing to skip, and a floating base makes that worse -- its six DOF
 * are ancestors of every joint, so the mass matrix has no sparsity for the
 * factorisation to exploit. But ABA's articulated-inertia congruence has a
 * fast path for joints whose origin carries no rotation, which most URDFs
 * give you. Measure your own model: the benchmark's `fd_crba` row against
 * `aba` is exactly this comparison.
 */
typedef enum {
    RD_FD_ABA = 0,
    RD_FD_CRBA
} rd_fd_method_t;

/**
 * @brief Floats of workspace rd_forward_dynamics() needs for a method.
 *
 * Zero for RD_FD_ABA. For RD_FD_CRBA it is nv*nv + nv: the mass matrix, then
 * the bias term, which is also where the right-hand side is built.
 */
rd_int_t rd_forward_dynamics_work(const rd_chain_t* chain, rd_fd_method_t method);

/**
 * @brief Forward dynamics by either recursion.
 *
 * @param tau      [nv] or NULL for zero torque.
 * @param work     [rd_forward_dynamics_work()] scratch, or NULL for RD_FD_ABA.
 *                 On return from RD_FD_CRBA the first nv*nv floats hold M(q)
 *                 row-major and the next nv hold h(q,qd) -- both are yours.
 * @param qdd_out  [nv].
 * @return RD_ERR_SINGULAR if the mass matrix is not positive definite, which
 *         means the model has a zero-inertia moving link.
 */
rd_status_t rd_forward_dynamics(const rd_chain_t* chain,
                                const rd_state_t* state,
                                const rd_real_t* tau,
                                const rd_real_t* gravity,
                                rd_fd_method_t method,
                                rd_real_t* work,
                                rd_real_t* qdd_out);

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
