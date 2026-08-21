/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file robot_dynamics.h
 * @brief Robot Dynamics Library - Main Header
 * 
 * Include this single header to use the entire library.
 * 
 * Features:
 * - Forward kinematics          (rd_fk_frame, rd_forward_kinematics)
 * - Geometric Jacobian          (rd_jacobian)
 * - Inverse dynamics via RNEA   (rd_rnea)
 * - Forward dynamics via ABA    (rd_aba)
 * - Mass matrix via CRBA        (rd_crba)
 * - Spatial velocity            (rd_spatial_velocity)
 * - Spatial acceleration        (rd_spatial_acceleration)
 * - Gravity / Coriolis terms    (rd_gravity, rd_nonlinear_terms, rd_coriolis)
 *
 * The library follows a model / chain / state split. A model is a static
 * description, a chain is its pre-processed form, and a state is the per-tick
 * workspace every algorithm reads. Call rd_update_kinematics() once per control
 * tick, then run as many algorithms as you like against it. Nothing here
 * allocates: all scratch lives in the state's caller-provided buffer.
 *
 * Usage:
 * @code
 * #include "robot_dynamics.h"
 *
 * rd_chain_t chain;
 * rd_chain_build(&my_model, &chain);
 *
 * static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
 * rd_state_t state;
 * rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));
 *
 * // Per control tick. q_base is NULL for a fixed-base robot; qd is packed to
 * // length nv, with the base twist in the first six elements when floating.
 * rd_update_kinematics(&chain, &state, q_base, q_joints, qd);
 *
 * rd_real_t tau[6 + RD_MAX_JOINTS];
 * rd_rnea(&chain, &state, qdd, NULL, tau);          // inverse dynamics
 *
 * rd_real_t qdd_out[6 + RD_MAX_JOINTS];
 * rd_aba(&chain, &state, tau, NULL, qdd_out);       // forward dynamics
 *
 * rd_real_t J[6 * (6 + RD_MAX_JOINTS)];
 * rd_jacobian(&chain, &state, frame_id, RD_FRAME_WORLD, J);
 *
 * rd_chain_free(&chain);
 * @endcode
 *
 * Configuration (define before including):
 * - RD_USE_SINGLE_PRECISION: Use float (default: 1)
 * - RD_USE_CMSIS_DSP: Use ARM CMSIS-DSP (default: 0)
 * - RD_MAX_LINKS / RD_MAX_JOINTS: model size bounds (default: 16 / 12)
 *
 * @version 0.5.0
 */

#ifndef ROBOT_DYNAMICS_H
#define ROBOT_DYNAMICS_H

#include "rd_state.h"

/* Configuration and types */
#include "rd_config.h"

/* Math utilities */
#include "rd_math.h"

/* Robot model definition */
#include "rd_model.h"

/* Kinematic chain */
#include "rd_chain.h"

/* Algorithms: FK, Jacobian, RNEA, CRBA, spatial acceleration/velocity */
#include "rd_algorithms.h"

#endif /* ROBOT_DYNAMICS_H */
