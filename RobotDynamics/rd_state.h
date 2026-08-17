/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_state.h
 * @brief Per-tick state cache and scratch workspace.
 *
 * One rd_state_t holds everything the algorithms share within a control tick:
 * the transforms and velocities rd_update_kinematics() computes once, plus the
 * scratch every algorithm needs while it runs. Because the scratch lives here,
 * no algorithm in this library allocates -- the whole control loop runs out of
 * a single caller-provided buffer.
 *
 * Vector conventions
 * ------------------
 * Configuration is split, because nq != nv for a floating base:
 *   q_base   [7]   x, y, z, qw, qx, qy, qz   (quaternion scalar FIRST)
 *   q_joints [nj]  one element per actuated joint
 *
 * Everything in velocity space is a single packed vector of length nv, with the
 * base occupying the first six elements when the model has a floating base.
 * This matches Pinocchio and bard, so qd/qdd/tau vectors are interchangeable
 * with them:
 *   qd, qdd, tau  [nv]   nv = 6 + nj (floating) or nj (fixed)
 *
 * Base twists and accelerations are expressed in the ROOT LINK'S BODY FRAME,
 * again matching Pinocchio's free-flyer convention.
 */

#ifndef RD_STATE_H
#define RD_STATE_H

#include <stddef.h>
#include "rd_config.h"
#include "rd_chain.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Buffer sizing
 * ============================================================================ */

/**
 * Floats of workspace per link. Cache is 60 (three 4x4 transforms, a spatial
 * velocity, a joint subspace); scratch is 62, sized by the largest consumer,
 * which is rd_aba().
 */
#define RD_STATE_FLOATS_PER_NODE  122

/** Elements to declare for a statically sized state buffer. */
#define RD_STATE_BUF_FLOATS(n)    ((n) * RD_STATE_FLOATS_PER_NODE + 16)

/* ============================================================================
 * State
 * ============================================================================ */

typedef struct {
    /* --- Cache: filled by rd_update_kinematics(), read by everything ------ */
    rd_real_t* T_parent_to_child; /**< 16*n, pose of each link in its parent */
    rd_real_t* Ti;                /**< 16*n, pose of each parent in its child */
    rd_real_t* T_world;           /**< 16*n, pose of each link in the world */
    rd_real_t* v;                 /**< 6*n, spatial velocity, link body frame */
    rd_real_t* S;                 /**< 6*n, joint motion subspace */

    /* --- Scratch: owned by whichever algorithm is running ----------------- */
    rd_real_t* inertia;           /**< 36*n, CRBA composite / ABA articulated */
    rd_real_t* accel;             /**< 6*n,  RNEA and ABA link accelerations */
    rd_real_t* force;             /**< 6*n,  RNEA forces / ABA bias forces */
    rd_real_t* cvp;               /**< 6*n,  ABA velocity-product accelerations */
    rd_real_t* U;                 /**< 6*n,  ABA */
    rd_real_t* D;                 /**< n,    ABA */
    rd_real_t* u;                 /**< n,    ABA */

    rd_int_t n_nodes;
} rd_state_t;

/* ============================================================================
 * API
 * ============================================================================ */

/** Bytes of buffer rd_state_init() needs for an n-link model. */
size_t rd_state_buffer_size(rd_int_t n_nodes);

/**
 * @brief Point an rd_state_t at a caller-provided buffer.
 *
 * The buffer must be at least rd_state_buffer_size(n_nodes) bytes and aligned
 * for rd_real_t. Nothing is allocated; the state borrows the buffer for as long
 * as it is used.
 */
rd_status_t rd_state_init(rd_state_t* state, rd_int_t n_nodes,
                          void* buffer, size_t buffer_size);

/**
 * @brief Compute the shared kinematics for this tick. Call once per control
 *        loop, before any other algorithm.
 *
 * Fills T_parent_to_child, Ti, T_world, S and v.
 *
 * @param q_base   [7] base pose, or NULL to place the base at the identity.
 *                 Ignored for fixed-base models.
 * @param q_joints [nj] joint positions, or NULL for the zero configuration.
 * @param qd       [nv] packed velocity; for a floating base the first six
 *                 elements are the base twist in the root body frame. NULL
 *                 means at rest.
 */
rd_status_t rd_update_kinematics(const rd_chain_t* chain,
                                 rd_state_t* state,
                                 const rd_real_t* q_base,
                                 const rd_real_t* q_joints,
                                 const rd_real_t* qd);

#ifdef __cplusplus
}
#endif

#endif /* RD_STATE_H */
