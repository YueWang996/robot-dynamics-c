/**
 * @file robot_dynamics.h
 * @brief Robot Dynamics Library - Main Header
 * 
 * Include this single header to use the entire library.
 * 
 * Features:
 * - Forward kinematics          (rd_fk_frame)
 * - Geometric Jacobian          (rd_jacobian_cached)
 * - Inverse dynamics via RNEA   (rd_rnea_cached)
 * - Mass matrix via CRBA        (rd_crba_cached)
 * - Spatial acceleration        (rd_spatial_acceleration_cached)
 * - Spatial velocity            (rd_get_spatial_velocity_cached)
 *
 * The library follows a model / chain / state split. A model is a static
 * description, a chain is the pre-processed form of it, and a state is the
 * per-tick cache of transforms and velocities that every algorithm reads.
 * Call rd_update_kinematics() once per control tick, then run as many
 * algorithms as you like against the cache.
 *
 * Usage:
 * @code
 * #include "robot_dynamics.h"
 *
 * rd_chain_t chain;
 * rd_chain_build(&my_model, &chain);
 *
 * // One state buffer, reused forever -- no allocation in the control loop
 * static rd_real_t buf[RD_MAX_LINKS * 66 + 16];
 * rd_state_t state;
 * rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));
 *
 * // Per control tick:
 * rd_update_kinematics(&chain, &state, q_joints, qd_joints);
 *
 * rd_real_t tau[RD_MAX_JOINTS + 6];
 * rd_rnea_cached(&chain, &state, NULL, qdd_joints, NULL, tau);
 *
 * rd_real_t J[6 * (RD_MAX_JOINTS + 6)];
 * rd_jacobian_cached(&chain, &state, frame_id, RD_FRAME_WORLD, J);
 *
 * rd_chain_free(&chain);
 * @endcode
 *
 * Configuration (define before including):
 * - RD_USE_SINGLE_PRECISION: Use float (default: 1)
 * - RD_USE_CMSIS_DSP: Use ARM CMSIS-DSP (default: 0)
 * - RD_USE_STATIC_ALLOC: Static allocation (default: 0)
 * - RD_MAX_LINKS / RD_MAX_JOINTS: model size bounds (default: 16 / 12)
 *
 * @version 1.0
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
