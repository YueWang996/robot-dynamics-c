/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file rd_algorithms.c
 * @brief Kinematics and dynamics implementation.
 *
 * All scratch comes from rd_state_t, so nothing here allocates.
 */

#include "rd_algorithms.h"
#include "rd_math.h"
#include <string.h>

/* ============================================================================
 * Internal helpers
 * ============================================================================ */

/* Gravity as a spatial motion vector in world coordinates: [-g; 0]. */
static RD_INLINE void algo_gravity_world(const rd_real_t* gravity, rd_real_t out[6]) {
    rd_real_t g[3] = {RD_REAL(0.0), RD_REAL(0.0), -RD_GRAVITY};
    if (gravity) { g[0] = gravity[0]; g[1] = gravity[1]; g[2] = gravity[2]; }
    out[0] = -g[0]; out[1] = -g[1]; out[2] = -g[2];
    out[3] = out[4] = out[5] = RD_REAL(0.0);
}

/* Velocity index of a node's joint, or -1 if it has none. */
/*
 * A frame's pose in the world, composed down its ancestry.
 *
 * rd_update_kinematics() does not keep world poses: they cost a 4x4 compose
 * per moving link per tick, and the algorithms that dominate a control loop --
 * RNEA, CRBA, ABA -- never read one. They work entirely in T_dyn, each link's
 * pose in its nearest moving ancestor. So the world pose is composed here, for
 * the frames actually asked about, at one compose per moving link on the path
 * rather than per moving link in the robot. A fixed link's own T_dyn closes
 * the chain: it is already expressed in its nearest moving ancestor, which is
 * where the walk stops.
 *
 * Deliberately not inlined -- four call sites, none of them in a loop.
 */
static void algo_frame_world(const rd_chain_t* chain,
                             const rd_state_t* state,
                             rd_idx_t frame, rd_real_t T_out[16]) {
    const rd_idx_t* path = &chain->parent_path[frame * chain->n_nodes];
    const rd_int_t  plen = chain->parent_path_len[frame];
    rd_real_t scratch[16];
    rd_real_t* acc = T_out;
    rd_real_t* alt = scratch;
    int have = 0;

    for (rd_int_t pi = 0; pi < plen; ++pi) {
        rd_idx_t node = path[pi];
        if (!rd_chain_node_is_dynamic(chain, node)) continue;
        if (!have) {
            memcpy(acc, &state->T_dyn[node*16], 16*sizeof(rd_real_t));
            have = 1;
        } else {
            rd_mat4_mul_se3(acc, &state->T_dyn[node*16], alt);
            rd_real_t* t = acc; acc = alt; alt = t;
        }
    }
    if (!rd_chain_node_is_dynamic(chain, frame)) {
        rd_mat4_mul_se3(acc, &state->T_dyn[frame*16], alt);
        rd_real_t* t = acc; acc = alt; alt = t;
    }
    if (acc != T_out) memcpy(T_out, acc, 16*sizeof(rd_real_t));
}

/*
 * A spatial motion vector, carried from a frame's own coordinates out to the
 * world.
 *
 * Ad(A B) v = Ad(A) (Ad(B) v), so the same ancestry the pose walk composes can
 * be applied one link at a time to the vector instead. That trades a 4x4
 * compose per step -- 36 multiplies and a sixteen-float temporary -- for a
 * transform of the six-vector itself, and needs no path array: dyn_parent
 * walks straight up.
 */
static void algo_frame_world_motion(const rd_chain_t* chain,
                                    const rd_state_t* state,
                                    rd_idx_t frame,
                                    const rd_real_t v_local[6],
                                    rd_real_t v_out[6]) {
    rd_real_t buf[6];
    rd_real_t* cur = buf;
    rd_real_t* nxt = v_out;
    memcpy(cur, v_local, 6*sizeof(rd_real_t));

    for (rd_idx_t node = frame; ; ) {
        rd_spatial_transform_motion(&state->T_dyn[node*16], cur, nxt);
        rd_real_t* t = cur; cur = nxt; nxt = t;
        rd_idx_t up = chain->dyn_parent[node];
        if (up == -1) break;
        node = up;
    }
    if (cur != v_out) memcpy(v_out, cur, 6*sizeof(rd_real_t));
}

static RD_INLINE void algo_frame_velocity(const rd_chain_t* chain,
                                          const rd_state_t* state,
                                          rd_idx_t frame, rd_real_t v_out[6]) {
    if (rd_chain_node_is_dynamic(chain, frame)) {
        memcpy(v_out, &state->v[frame*6], 6*sizeof(rd_real_t));
    } else {
        rd_spatial_transform_motion_inv(&state->T_dyn[frame*16],
                                        &state->v[chain->dyn_parent[frame]*6],
                                        v_out);
    }
}

static RD_INLINE rd_int_t algo_vel_index(const rd_chain_t* chain, rd_idx_t node) {
    rd_idx_t jidx = chain->joint_idx[node];
    if (jidx < 0 || chain->joint_type[node] == RD_JOINT_FIXED ||
        chain->joint_type[node] == RD_JOINT_FLOATING) {
        return -1;
    }
    return chain->has_floating_base ? (6 + jidx) : jidx;
}

rd_status_t rd_cholesky_factor(rd_real_t* RD_RESTRICT A, rd_int_t n,
                               rd_real_t* RD_RESTRICT dinv) {
    if (!A || !dinv) return RD_ERR_NULL_PTR;
    for (rd_int_t j = 0; j < n; ++j) {
        rd_real_t d = A[j*n + j];
        for (rd_int_t k = 0; k < j; ++k) d -= A[j*n + k] * A[j*n + k];
        if (d <= RD_EPS) return RD_ERR_SINGULAR;  /* not positive definite */
        d = rd_sqrt(d);
        A[j*n + j] = d;
        const rd_real_t inv = RD_REAL(1.0) / d;
        dinv[j] = inv;                            /* both substitutions want it */
        for (rd_int_t i = j + 1; i < n; ++i) {
            rd_real_t sum = A[i*n + j];
            for (rd_int_t k = 0; k < j; ++k) sum -= A[i*n + k] * A[j*n + k];
            A[i*n + j] = sum * inv;
        }
    }
    return RD_OK;
}

rd_status_t rd_cholesky_solve(const rd_real_t* RD_RESTRICT L,
                              const rd_real_t* RD_RESTRICT dinv,
                              const rd_real_t* RD_RESTRICT b,
                              rd_real_t* RD_RESTRICT x, rd_int_t n) {
    if (!L || !dinv || !b || !x) return RD_ERR_NULL_PTR;
    /* forward substitution: L y = b, into x */
    for (rd_int_t i = 0; i < n; ++i) {
        rd_real_t sum = b[i];
        for (rd_int_t k = 0; k < i; ++k) sum -= L[i*n + k] * x[k];
        x[i] = sum * dinv[i];
    }
    /* back substitution: L^T x = y */
    for (rd_int_t i = n - 1; i >= 0; --i) {
        rd_real_t sum = x[i];
        for (rd_int_t k = i + 1; k < n; ++k) sum -= L[k*n + i] * x[k];
        x[i] = sum * dinv[i];
    }
    return RD_OK;
}

/* Factor and solve in one go, which is what the forward dynamics wants. */
static int algo_solve_spd(rd_real_t* RD_RESTRICT L, const rd_real_t* RD_RESTRICT b,
                          rd_real_t* RD_RESTRICT x, rd_int_t n,
                          rd_real_t* RD_RESTRICT dinv) {
    if (rd_cholesky_factor(L, n, dinv) != RD_OK) return -1;
    rd_cholesky_solve(L, dinv, b, x, n);
    return 0;
}

#if RD_ENABLE_ABA
/*
 * The floating base's 6x6 block. Same factorisation, with the outer loop
 * written out so that every trip count below it is a literal. rd_aba()'s root
 * solve is the only caller, so it goes with it.
 *
 * That is the whole point: GCC will not peel this nest on its own -- its
 * budget for complete unrolling is 200 instructions and the nest needs about
 * 800 -- so it leaves nine loops in place and their overhead dominates, 830
 * instructions executed for 121 multiply-accumulates. Unrolled it is 191
 * instructions with no loops at all, which is also *smaller* than the rolled
 * code it replaces.
 *
 * Destroys L, which is the caller's temporary; there is no copy.
 */
#define RD_CHOL6_(J)                                                          \
    do {                                                                      \
        rd_real_t d_ = L[(J)*6 + (J)];                                        \
        for (int k_ = 0; k_ < (J); ++k_) d_ -= L[(J)*6+k_] * L[(J)*6+k_];     \
        if (d_ <= RD_EPS) return -1;              /* not positive definite */ \
        d_ = rd_sqrt(d_);                                                     \
        L[(J)*6 + (J)] = d_;                                                  \
        const rd_real_t iv_ = RD_REAL(1.0) / d_;                              \
        dinv[(J)] = iv_;                                                      \
        for (int i_ = (J)+1; i_ < 6; ++i_) {                                  \
            rd_real_t s_ = L[i_*6 + (J)];                                     \
            for (int k_ = 0; k_ < (J); ++k_) s_ -= L[i_*6+k_] * L[(J)*6+k_];  \
            L[i_*6 + (J)] = s_ * iv_;                                         \
        }                                                                     \
    } while (0)

static int algo_solve6_spd(rd_real_t L[36], const rd_real_t b[6], rd_real_t x[6]) {
    rd_real_t dinv[6];
    RD_CHOL6_(0); RD_CHOL6_(1); RD_CHOL6_(2);
    RD_CHOL6_(3); RD_CHOL6_(4); RD_CHOL6_(5);

    for (int i = 0; i < 6; ++i) {                 /* L y = b, into x */
        rd_real_t s = b[i];
        for (int k = 0; k < i; ++k) s -= L[i*6 + k] * x[k];
        x[i] = s * dinv[i];
    }
    for (int i = 5; i >= 0; --i) {                /* L^T x = y */
        rd_real_t s = x[i];
        for (int k = i + 1; k < 6; ++k) s -= L[k*6 + i] * x[k];
        x[i] = s * dinv[i];
    }
    return 0;
}

#undef RD_CHOL6_
#endif /* RD_ENABLE_ABA */

/* ============================================================================
 * Kinematics
 * ============================================================================ */

rd_status_t rd_fk_frame(const rd_chain_t* chain,
                        const rd_real_t* q_base,
                        const rd_real_t* q_joints,
                        rd_idx_t frame_id,
                        rd_real_t T_out[16]) {
    if (!chain || !T_out) return RD_ERR_NULL_PTR;
    if (frame_id < 0 || frame_id >= chain->n_nodes) return RD_ERR_INVALID_INDEX;

    if (chain->has_floating_base && q_base) {
        rd_real_t Rb[9];
        rd_rot_quat(&q_base[3], Rb);
        rd_rot_to_mat4(Rb, q_base, T_out);
    } else {
        rd_mat4_identity(T_out);
    }

    rd_int_t plen = chain->parent_path_len[frame_id];
    const rd_idx_t* path = &chain->parent_path[frame_id * chain->n_nodes];
    rd_real_t Ttmp1[16];

    for (rd_int_t pi = 0; pi < plen; ++pi) {
        rd_idx_t node = path[pi];
        rd_mat4_mul_se3(T_out, &chain->T_joint_offset[node*16], Ttmp1);

        rd_idx_t jidx = chain->joint_idx[node];
        if (jidx >= 0 && q_joints) {
            rd_mat4_mul_joint(Ttmp1, chain->s_axis[node], chain->s_sign[node],
                              q_joints[jidx], T_out);
        } else {
            memcpy(T_out, Ttmp1, 16*sizeof(rd_real_t));
        }
    }
    return RD_OK;
}

rd_status_t rd_forward_kinematics(const rd_chain_t* chain,
                                  const rd_state_t* state,
                                  rd_idx_t frame_id,
                                  rd_real_t T_out[16]) {
    if (!chain || !state || !T_out) return RD_ERR_NULL_PTR;
    if (frame_id < 0 || frame_id >= chain->n_nodes) return RD_ERR_INVALID_INDEX;
    algo_frame_world(chain, state, frame_id, T_out);
    return RD_OK;
}

rd_status_t rd_jacobian(const rd_chain_t* chain,
                        const rd_state_t* state,
                        rd_idx_t frame_id,
                        rd_frame_t ref_frame,
                        rd_real_t* J_out) {
    if (!chain || !state || !J_out) return RD_ERR_NULL_PTR;
    if (frame_id < 0 || frame_id >= chain->n_nodes) return RD_ERR_INVALID_INDEX;

    const rd_int_t nv = rd_chain_get_nv(chain);
    rd_zero(J_out, 6 * nv);

    /* Base columns: a unit twist in the root body frame, mapped to the world.
     * The root's pose in its nearest moving ancestor is its pose in the world. */
    if (chain->has_floating_base) {
        const rd_real_t* T_base = &state->T_dyn[chain->topo_order[0]*16];
        rd_real_t col[6];
        for (rd_int_t k = 0; k < 6; ++k) {
            rd_spatial_transform_motion_axis(T_base, k, RD_REAL(1.0), col);
            for (int r = 0; r < 6; ++r) J_out[r*nv + k] = col[r];
        }
    }

    rd_int_t plen = chain->parent_path_len[frame_id];
    const rd_idx_t* path = &chain->parent_path[frame_id * chain->n_nodes];

    /*
     * One walk down the path does everything: the running product of T_dyn is
     * the world pose of each link as it is reached, which is what that link's
     * column needs, and what is left in it at the end is the frame's own pose
     * for the local-frame rotation below.
     */
    rd_real_t Tacc[16], Talt[16];
    rd_real_t* acc = Tacc;
    rd_real_t* alt = Talt;
    int have = 0;

    for (rd_int_t pi = 0; pi < plen; ++pi) {
        rd_idx_t node = path[pi];
        if (!rd_chain_node_is_dynamic(chain, node)) continue;

        if (!have) {
            memcpy(acc, &state->T_dyn[node*16], 16*sizeof(rd_real_t));
            have = 1;
        } else {
            rd_mat4_mul_se3(acc, &state->T_dyn[node*16], alt);
            rd_real_t* t = acc; acc = alt; alt = t;
        }

        rd_int_t col_idx = algo_vel_index(chain, node);
        if (col_idx < 0) continue;

        rd_real_t col[6];
        rd_spatial_transform_motion_axis(acc, chain->s_axis[node],
                                         chain->s_sign[node], col);
        for (int r = 0; r < 6; ++r) J_out[r*nv + col_idx] = col[r];
    }

    if (ref_frame == RD_FRAME_LOCAL) {
        if (!rd_chain_node_is_dynamic(chain, frame_id)) {
            rd_mat4_mul_se3(acc, &state->T_dyn[frame_id*16], alt);
            rd_real_t* t = acc; acc = alt; alt = t;
        }
        rd_real_t T_f_w[16];
        rd_mat4_inv(acc, T_f_w);

        /*
         * Only the columns the passes above wrote can be nonzero -- the base
         * block, and the joints on this frame's own path. Every other column
         * belongs to a joint that cannot move this frame, and rotating it
         * would be transforming zeros. On Go2 that is nine columns of
         * eighteen.
         */
        #define RD_J_LOCAL_COL(c) do {                                        \
            rd_real_t in_[6], out_[6];                                        \
            for (int r_ = 0; r_ < 6; ++r_) in_[r_] = J_out[r_*nv + (c)];      \
            rd_spatial_transform_motion(T_f_w, in_, out_);                    \
            for (int r_ = 0; r_ < 6; ++r_) J_out[r_*nv + (c)] = out_[r_];     \
        } while (0)

        if (chain->has_floating_base) {
            for (rd_int_t k = 0; k < 6; ++k) RD_J_LOCAL_COL(k);
        }
        for (rd_int_t pi = 0; pi < plen; ++pi) {
            rd_int_t ci = algo_vel_index(chain, path[pi]);
            if (ci >= 0) RD_J_LOCAL_COL(ci);
        }
        #undef RD_J_LOCAL_COL
    }
    return RD_OK;
}

rd_status_t rd_spatial_velocity(const rd_chain_t* chain,
                                const rd_state_t* state,
                                rd_idx_t frame_id,
                                rd_frame_t ref_frame,
                                rd_real_t v_out[6]) {
    if (!chain || !state || !v_out) return RD_ERR_NULL_PTR;
    if (frame_id < 0 || frame_id >= chain->n_nodes) return RD_ERR_INVALID_INDEX;

    rd_real_t v_local[6];
    algo_frame_velocity(chain, state, frame_id, v_local);
    if (ref_frame == RD_FRAME_LOCAL) {
        memcpy(v_out, v_local, 6 * sizeof(rd_real_t));
    } else {
        algo_frame_world_motion(chain, state, frame_id, v_local, v_out);
    }
    return RD_OK;
}

rd_status_t rd_spatial_acceleration(const rd_chain_t* chain,
                                    const rd_state_t* state,
                                    const rd_real_t* qdd,
                                    rd_idx_t frame_id,
                                    rd_frame_t ref_frame,
                                    rd_real_t a_out[6]) {
    if (!chain || !state || !a_out) return RD_ERR_NULL_PTR;
    if (frame_id < 0 || frame_id >= chain->n_nodes) return RD_ERR_INVALID_INDEX;

    rd_real_t* a = state->accel;

    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        const rd_dyn_node_t* d = &chain->dyn[di];
        const rd_idx_t node   = d->node;
        const rd_idx_t parent = d->danc;
        const rd_real_t* Td  = &state->T_dyn[node*16];
        const rd_real_t* v_i = &state->v[node*6];

        /* No gravity term: this is a true spatial acceleration, not the
         * gravity-loaded pseudo-acceleration RNEA propagates. */
        rd_real_t a_p[6];
        if (parent == -1) {
            if (chain->has_floating_base && qdd) {
                memcpy(a_p, qdd, 6*sizeof(rd_real_t));
            } else {
                memset(a_p, 0, 6*sizeof(rd_real_t));
            }
        } else {
            rd_spatial_transform_motion_inv(Td, &a[parent*6], a_p);
        }

        rd_real_t a_joint = (d->vidx >= 0 && qdd) ? qdd[d->vidx] : RD_REAL(0.0);

        rd_real_t cor[6];
        rd_spatial_cross_axis(v_i, d->s_axis, d->s_sign * state->vj[node], cor);

        for (int k = 0; k < 6; ++k) a[node*6 + k] = a_p[k] + cor[k];
        a[node*6 + d->s_axis] += d->s_sign * a_joint;
    }

    /* A fixed link is rigidly attached to its moving ancestor, so its
     * acceleration is that one transformed -- no joint or Coriolis term. */
    rd_real_t a_f[6];
    if (rd_chain_node_is_dynamic(chain, frame_id)) {
        memcpy(a_f, &a[frame_id*6], 6*sizeof(rd_real_t));
    } else {
        rd_spatial_transform_motion_inv(&state->T_dyn[frame_id*16],
                                        &a[chain->dyn_parent[frame_id]*6], a_f);
    }
    if (ref_frame == RD_FRAME_WORLD) {
        algo_frame_world_motion(chain, state, frame_id, a_f, a_out);
    } else {
        memcpy(a_out, a_f, 6*sizeof(rd_real_t));
    }
    return RD_OK;
}

/* ============================================================================
 * Inverse dynamics
 * ============================================================================ */

/*
 * Seed a per-node force array with -f_ext, gathered onto the dynamics tree.
 *
 * f_ext is indexed by node and expressed in each link's own body frame. A
 * contact usually lands on a *fixed* link -- a foot, a fingertip, a sensor pad
 * -- and rd_chain_build folded those into the moving node above them, so the
 * force has to be carried there first. state->T_dyn holds a fixed node's pose
 * in exactly that node, which is the transform this needs.
 *
 * The result is negated because the inward recursion carries
 * f_i = I a_i + v x* I v_i - f_i^ext, and the caller adds its own terms on top
 * of what this leaves behind. Links with no force cost six comparisons.
 */
static void algo_seed_fext(const rd_chain_t* chain, const rd_state_t* state,
                           const rd_real_t* RD_RESTRICT f_ext,
                           rd_real_t* RD_RESTRICT f) {
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_zero(&f[chain->dyn[di].node * 6], 6);
    }
    for (rd_int_t nd = 0; nd < chain->n_nodes; ++nd) {
        const rd_real_t* fe = &f_ext[nd * 6];
        int any = 0;
        for (int k = 0; k < 6; ++k) any |= (fe[k] != RD_REAL(0.0));
        if (!any) continue;

        const int own = rd_chain_node_is_dynamic(chain, (rd_idx_t)nd);
        const rd_idx_t tgt = own ? (rd_idx_t)nd : chain->dyn_parent[nd];
        if (tgt < 0) continue;          /* anchored to the world; the ground takes it */

        rd_real_t neg[6];
        for (int k = 0; k < 6; ++k) neg[k] = -fe[k];
        if (own) {
            for (int k = 0; k < 6; ++k) f[nd * 6 + k] += neg[k];
        } else {
            rd_spatial_transform_force_accum(&state->T_dyn[nd * 16], neg,
                                             &f[tgt * 6]);
        }
    }
}

/*
 * Shared RNEA body. With use_velocity = 0 the cached link velocities are
 * treated as zero, which drops the Coriolis and bias terms and leaves
 * tau = M(q) qdd + g(q) -- that is how rd_gravity() gets g(q) without needing
 * a second rd_update_kinematics() at qd = 0.
 *
 * Always inlined, and called from exactly two places with a literal, so the
 * two recursions are compiled separately and neither carries a test the other
 * needs. rd_gravity() is a control-tick call in its own right, and the flag it
 * passes turns into six adds of a zeroed vector per link if it stays a
 * variable.
 */
static RD_INLINE rd_status_t rnea_body(const rd_chain_t* chain,
                                       const rd_state_t* state,
                                       const rd_real_t* qdd,
                                       const rd_real_t* gravity,
                                       int use_velocity,
                                       rd_real_t* tau_out) {
    if (!chain || !state || !tau_out) return RD_ERR_NULL_PTR;

    const rd_int_t nv = rd_chain_get_nv(chain);
    rd_real_t* a = state->accel;
    rd_real_t* f = state->force;

    rd_real_t g_world[6];
    algo_gravity_world(gravity, g_world);

    rd_zero(tau_out, nv);

    /* Outward: accelerations. Over the dynamics tree, so fixed links -- whose
     * inertia rd_chain_build already folded into the moving node above them --
     * are never visited. */
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        const rd_dyn_node_t* d = &chain->dyn[di];
        const rd_idx_t node   = d->node;
        const rd_idx_t parent = d->danc;
        const rd_real_t* Td  = &state->T_dyn[node*16];

        rd_real_t a_p[6];
        if (parent == -1) {
            /* Puts gravity in the root body frame, where the caller's base
             * acceleration already is. */
            rd_spatial_transform_motion_inv(Td, g_world, a_p);
            if (chain->has_floating_base && qdd) {
                for (int k = 0; k < 6; ++k) a_p[k] += qdd[k];
            }
        } else {
            rd_spatial_transform_motion_inv(Td, &a[parent*6], a_p);
        }

        rd_real_t a_joint = (d->vidx >= 0 && qdd) ? qdd[d->vidx] : RD_REAL(0.0);

        rd_real_t cor[6];
        if (use_velocity) {
            rd_spatial_cross_axis(&state->v[node*6], d->s_axis,
                                  d->s_sign * state->vj[node], cor);
        } else {
            memset(cor, 0, 6*sizeof(rd_real_t));
        }

        for (int k = 0; k < 6; ++k) a[node*6 + k] = a_p[k] + cor[k];
        a[node*6 + d->s_axis] += d->s_sign * a_joint;
    }

    /* Inward: forces */
    for (rd_int_t di = chain->n_dyn - 1; di >= 0; --di) {
        rd_idx_t node = chain->dyn[di].node;
        rd_real_t* f_i = &f[node*6];
        const rd_real_t* I_i = &chain->inertia_compact[node*RD_INERTIA_COMPACT_LEN];

        rd_real_t Ia[6];
        rd_spatial_inertia_mul(I_i, &a[node*6], Ia);

        if (use_velocity) {
            rd_real_t Iv[6], bias[6];
            rd_spatial_inertia_mul(I_i, &state->v[node*6], Iv);
            rd_spatial_cross_force(&state->v[node*6], Iv, bias);
            for (int k = 0; k < 6; ++k) f_i[k] = Ia[k] + bias[k];
        } else {
            for (int k = 0; k < 6; ++k) f_i[k] = Ia[k];
        }

        const rd_int_t c0 = chain->dyn_child_start[node];
        const rd_int_t c1 = chain->dyn_child_start[node + 1];
        for (rd_int_t ci = c0; ci < c1; ++ci) {
            rd_idx_t child = chain->dyn_child[ci];
            rd_spatial_transform_force_accum(&state->T_dyn[child*16],
                                             &f[child*6], f_i);
        }
    }

    /* Project onto the joint axes */
    rd_idx_t root = chain->topo_order[0];
    if (chain->has_floating_base) {
        for (int k = 0; k < 6; ++k) tau_out[k] = f[root*6 + k];
    }
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        const rd_dyn_node_t* d = &chain->dyn[di];
        if (d->vidx < 0) continue;
        tau_out[d->vidx] = d->s_sign * f[d->node*6 + d->s_axis];
        if (qdd) tau_out[d->vidx] += chain->armature[d->vidx] * qdd[d->vidx];
    }
    return RD_OK;
}

/* The two the flag selects, each compiled on its own so that neither carries
 * a test only the other needs. */
static RD_NOINLINE rd_status_t rnea_with_velocity(
        const rd_chain_t* chain, const rd_state_t* state, const rd_real_t* qdd,
        const rd_real_t* gravity, rd_real_t* tau_out) {
    return rnea_body(chain, state, qdd, gravity, 1, tau_out);
}

static RD_NOINLINE rd_status_t rnea_no_velocity(
        const rd_chain_t* chain, const rd_state_t* state, const rd_real_t* qdd,
        const rd_real_t* gravity, rd_real_t* tau_out) {
    return rnea_body(chain, state, qdd, gravity, 0, tau_out);
}

rd_status_t rd_rnea(const rd_chain_t* chain, const rd_state_t* state,
                    const rd_real_t* qdd, const rd_real_t* gravity,
                    rd_real_t* tau_out) {
    return rnea_with_velocity(chain, state, qdd, gravity, tau_out);
}

/*
 * tau -= sum_i J_i^T f_ext_i, as an inward pass of its own.
 *
 * RNEA is linear in the external forces, so this is separable, and separating
 * it is what keeps rd_rnea() exactly the function it was: threading f_ext
 * through the main recursion put a test inside its inner loop and cost 4-5% of
 * every call that passed NULL. Here nobody pays for it who does not use it.
 *
 * The recursion is the inward half of RNEA with the inertial terms dropped:
 * each link's force is already in state->force from the seeding, children are
 * carried into their parent, and the joint axes read off what is left.
 */
static void algo_fext_torque(const rd_chain_t* chain, const rd_state_t* state,
                             const rd_real_t* f_ext, rd_real_t* tau_out) {
    rd_real_t* f = state->force;
    algo_seed_fext(chain, state, f_ext, f);

    for (rd_int_t di = chain->n_dyn - 1; di >= 0; --di) {
        const rd_idx_t node = chain->dyn[di].node;
        const rd_int_t c0 = chain->dyn_child_start[node];
        const rd_int_t c1 = chain->dyn_child_start[node + 1];
        for (rd_int_t ci = c0; ci < c1; ++ci) {
            const rd_idx_t child = chain->dyn_child[ci];
            rd_spatial_transform_force_accum(&state->T_dyn[child*16],
                                             &f[child*6], &f[node*6]);
        }
    }

    if (chain->has_floating_base) {
        const rd_idx_t root = chain->topo_order[0];
        for (int k = 0; k < 6; ++k) tau_out[k] += f[root*6 + k];
    }
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        const rd_dyn_node_t* d = &chain->dyn[di];
        if (d->vidx < 0) continue;
        tau_out[d->vidx] += d->s_sign * f[d->node*6 + d->s_axis];
    }
}

rd_status_t rd_rnea_ext(const rd_chain_t* chain, const rd_state_t* state,
                        const rd_real_t* qdd, const rd_real_t* gravity,
                        const rd_real_t* f_ext, rd_real_t* tau_out) {
    rd_status_t st = rnea_with_velocity(chain, state, qdd, gravity, tau_out);
    if (st != RD_OK || !f_ext) return st;
    algo_fext_torque(chain, state, f_ext, tau_out);
    return RD_OK;
}

/* ============================================================================
 * Forward dynamics, either way
 * ============================================================================ */

rd_int_t rd_forward_dynamics_work(const rd_chain_t* chain, rd_fd_method_t method) {
    if (!chain || method != RD_FD_CRBA) return 0;
    const rd_int_t nv = rd_chain_get_nv(chain);
    return nv * nv + nv;
}

rd_status_t rd_forward_dynamics(const rd_chain_t* chain,
                                const rd_state_t* state,
                                const rd_real_t* tau,
                                const rd_real_t* gravity,
                                rd_fd_method_t method,
                                rd_real_t* work,
                                rd_real_t* qdd_out) {
    if (!chain || !state || !qdd_out) return RD_ERR_NULL_PTR;
#if RD_ENABLE_ABA
    if (method == RD_FD_ABA) return rd_aba(chain, state, tau, gravity, qdd_out);
#endif
    /* RD_FD_ABA lands here when it is compiled out, and is rejected with
     * every other value that is not a method. */
    if (method != RD_FD_CRBA) return RD_ERR_INVALID_INDEX;
    if (!work) return RD_ERR_NULL_PTR;

    const rd_int_t nv = rd_chain_get_nv(chain);
    rd_real_t* M = work;              /* nv*nv, row-major */
    rd_real_t* h = work + nv * nv;    /* nv, then reused as the right-hand side */

    rd_status_t st = rd_crba(chain, state, M);
    if (st != RD_OK) return st;
    st = rd_nonlinear_terms(chain, state, gravity, h);
    if (st != RD_OK) return st;

    /* M qdd = tau - h. The right-hand side goes over h; M is left holding its
     * own Cholesky factor, which is why the header promises the caller M only
     * as far as the factorisation. */
    for (rd_int_t k = 0; k < nv; ++k) {
        h[k] = (tau ? tau[k] : RD_REAL(0.0)) - h[k];
    }

    /* The reciprocal diagonal needs nv more floats and there is nowhere left
     * in work, so it comes off the stack, bounded by the same limit the rest
     * of the library uses for a velocity vector. */
    rd_real_t dinv[6 + RD_MAX_JOINTS];
    if (nv > (rd_int_t)(sizeof(dinv)/sizeof(dinv[0]))) return RD_ERR_INVALID_SIZE;

    if (algo_solve_spd(M, h, qdd_out, nv, dinv) != 0) return RD_ERR_SINGULAR;
    return RD_OK;
}

rd_status_t rd_gravity(const rd_chain_t* chain, const rd_state_t* state,
                       const rd_real_t* gravity, rd_real_t* tau_out) {
    return rnea_no_velocity(chain, state, NULL, gravity, tau_out);
}

rd_status_t rd_nonlinear_terms(const rd_chain_t* chain, const rd_state_t* state,
                               const rd_real_t* gravity, rd_real_t* tau_out) {
    return rnea_with_velocity(chain, state, NULL, gravity, tau_out);
}

rd_status_t rd_coriolis(const rd_chain_t* chain, const rd_state_t* state,
                        rd_real_t* tau_out) {
    rd_real_t zero_g[3] = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)};
    return rnea_with_velocity(chain, state, NULL, zero_g, tau_out);
}

/*
 * M's diagonal gains each joint's reflected rotor inertia: the rotor turns
 * with its own joint and nothing else.
 *
 * Out of line, and behind a flag, because it is not free. Walking a diagonal
 * of stride nv+1 costs 4% of rd_crba on Go2 and 5% on the G1 -- and merely
 * *having* the loop inside rd_crba cost 2-4% on models where the flag is off
 * and it never runs, since the function grows and rd_crba is fetch-bound at
 * four flash wait states. A model with no rotor inertia, which is every URDF
 * until someone sets one, now pays a load and a branch.
 */
static RD_NOINLINE void crba_add_armature(const rd_chain_t* chain,
                                          rd_real_t* RD_RESTRICT M_out,
                                          rd_int_t nv) {
    for (rd_int_t vi = 0; vi < nv; ++vi) M_out[vi*nv + vi] += chain->armature[vi];
}

/* ============================================================================
 * Mass matrix
 * ============================================================================ */

rd_status_t rd_crba(const rd_chain_t* chain, const rd_state_t* state,
                    rd_real_t* M_out) {
    if (!chain || !state || !M_out) return RD_ERR_NULL_PTR;

    const rd_int_t nv = rd_chain_get_nv(chain);
    /* The composite is a sum of rigid-body inertias in a common frame, which is
     * itself a rigid body, so it lives in ten numbers rather than thirty-six.
     * state->inertia is sized for the 21 that ABA's articulated inertia needs;
     * here only the first ten floats of each node's slot are used. */
    rd_real_t* Ic = state->inertia;
    #define RD_IC(n) (&Ic[(n) * RD_INERTIA_COMPACT_LEN])

    rd_zero(M_out, nv * nv);

    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node = chain->dyn[di].node;
        memcpy(RD_IC(node), &chain->inertia_compact[node*RD_INERTIA_COMPACT_LEN],
               RD_INERTIA_COMPACT_LEN * sizeof(rd_real_t));
    }

    /*
     * Inward: composite inertia. Takes the child-in-parent transform directly,
     * so there is no inverse to form per node.
     *
     * The two loops differ only in the axis argument, and a chain with no
     * axis-aligned joint runs the one where it is the literal -1, so the
     * axis-aligned kernel is not compiled into that loop at all. Leaving it
     * there to be branched over is not free: it takes registers away from the
     * general kernel, which then spends them on moves. That was worth 4% of
     * an xarm7 mass matrix, on a path it never executes.
     */
    if (chain->n_axis_rot == 0) {
        for (rd_int_t di = chain->n_dyn - 1; di >= 0; --di) {
            const rd_dyn_node_t* d = &chain->dyn[di];
            if (d->danc != -1) {
                rd_rbi_congruence_accum(&state->T_dyn[d->node*16], -1,
                                        RD_IC(d->node), RD_IC(d->danc));
            }
        }
    } else {
        for (rd_int_t di = chain->n_dyn - 1; di >= 0; --di) {
            const rd_dyn_node_t* d = &chain->dyn[di];
            if (d->danc != -1) {
                rd_rbi_congruence_accum(&state->T_dyn[d->node*16], d->axis_rot,
                                        RD_IC(d->node), RD_IC(d->danc));
            }
        }
    }

    rd_idx_t root = chain->topo_order[0];
    if (chain->has_floating_base) {
        rd_rbi_to_mat6_stride(RD_IC(root), M_out, nv);
    }

    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        const rd_dyn_node_t* d = &chain->dyn[di];
        const rd_idx_t node = d->node;
        const rd_int_t col = d->vidx;
        if (col < 0) continue;

        /*
         * Walk the parent chain once, filling both the joint-joint couplings
         * and, on reaching the root, the floating-base coupling. The force
         * moves up in place: each step overwrites the frame it came from.
         */
        rd_real_t f_prop[6];
        rd_rbi_col(RD_IC(node), d->s_axis, d->s_sign, f_prop);
        M_out[col*nv + col] = d->s_sign * f_prop[d->s_axis];

        const rd_dyn_node_t* dc = d;
        for (;;) {
            if (dc->danc == -1) break;

            rd_spatial_transform_force_inplace(&state->T_dyn[dc->node*16], f_prop);
            dc = &chain->dyn[chain->dyn_slot[dc->danc]];

            if (dc->vidx >= 0) {
                rd_real_t val = dc->s_sign * f_prop[dc->s_axis];
                M_out[col*nv + dc->vidx] = val;
                M_out[dc->vidx*nv + col] = val;
            }
        }

        if (chain->has_floating_base) {
            /* f_prop now sits in the root frame. */
            for (int k = 0; k < 6; ++k) {
                M_out[k*nv + col] = f_prop[k];
                M_out[col*nv + k] = f_prop[k];
            }
        }
    }
    if (chain->has_armature) crba_add_armature(chain, M_out, nv);

    #undef RD_IC
    return RD_OK;
}

/* ============================================================================
 * Forward dynamics (Articulated Body Algorithm)
 * ============================================================================ */
#if RD_ENABLE_ABA

static rd_status_t aba_impl(const rd_chain_t* chain, const rd_state_t* state,
                            const rd_real_t* tau, const rd_real_t* gravity,
                            const rd_real_t* f_ext, rd_real_t* qdd_out) {
    if (!chain || !state || !qdd_out) return RD_ERR_NULL_PTR;

    const rd_int_t nv = rd_chain_get_nv(chain);

    /* Symmetric throughout, so 21 numbers per node rather than 36. This is
     * what state->inertia is sized for; CRBA takes the first ten of each
     * slot. */
    rd_real_t* IA = state->inertia;
    #define RD_IA(n) (&IA[(n) * RD_ABI_LEN])
    rd_real_t* pA = state->force;     /* bias force,               6*n  */
    rd_real_t* c  = state->cvp;       /* velocity-product accel,   6*n  */
    rd_real_t* U  = state->U;
    rd_real_t* D  = state->D;
    rd_real_t* u  = state->u;
    rd_real_t* a  = state->accel;

    rd_zero(qdd_out, nv);
    if (f_ext) algo_seed_fext(chain, state, f_ext, pA);

    /* --- Pass 1 (outward): bias forces and velocity-product accelerations -- */
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        const rd_dyn_node_t* d = &chain->dyn[di];
        const rd_idx_t node = d->node;
        const rd_real_t* v_i = &state->v[node*6];

        rd_abi_from_rbi(&chain->inertia_compact[node*RD_INERTIA_COMPACT_LEN],
                        RD_IA(node));

        rd_spatial_cross_axis(v_i, d->s_axis, d->s_sign * state->vj[node],
                              &c[node*6]);

        rd_real_t Iv[6];
        rd_spatial_inertia_mul(&chain->inertia_compact[node*RD_INERTIA_COMPACT_LEN],
                               v_i, Iv);
        if (f_ext) {
            /* pA already holds -f_ext for this node. */
            rd_real_t bias[6];
            rd_spatial_cross_force(v_i, Iv, bias);
            for (int k = 0; k < 6; ++k) pA[node*6 + k] += bias[k];
        } else {
            rd_spatial_cross_force(v_i, Iv, &pA[node*6]);
        }
    }

    /* --- Pass 2 (inward): articulate the inertias -------------------------- */
    for (rd_int_t di = chain->n_dyn - 1; di >= 0; --di) {
        const rd_dyn_node_t* d = &chain->dyn[di];
        const rd_idx_t node   = d->node;
        const rd_idx_t parent = d->danc;
        const rd_int_t   sk = d->s_axis;
        const rd_real_t  sg = d->s_sign;
        const rd_int_t vi = d->vidx;

        /*
         * The rank-1 downdate happens in place: IA[node] is dead once this
         * link's contribution has been pushed to its parent, so copying it
         * aside first would just be 36 floats of pointless traffic.
         */
        rd_real_t* Ia = RD_IA(node);
        rd_real_t* pa = &pA[node*6];

        if (vi >= 0) {
            rd_abi_col(Ia, sk, sg, &U[node*6]);
            rd_real_t d = sg * U[node*6 + sk] + chain->armature[vi];
            if (d <= RD_EPS) return RD_ERR_SINGULAR;
            D[node] = d;
            u[node] = (tau ? tau[vi] : RD_REAL(0.0)) - sg * pa[sk];

            /* Ia -= U D^-1 U^T */
            rd_real_t inv_d = RD_REAL(1.0) / d;
            rd_abi_rank1_sub(Ia, &U[node*6], inv_d);
            /* pa += Ia c + U D^-1 u */
            rd_abi_mul_accum(Ia, &c[node*6], pa);
            rd_real_t ud = u[node] * inv_d;
            for (int k = 0; k < 6; ++k) pa[k] += U[node*6 + k] * ud;
        } else {
            D[node] = RD_REAL(0.0);
            u[node] = RD_REAL(0.0);
            /* A rigid connection: c is zero, so pa is unchanged. */
            rd_abi_mul_accum(Ia, &c[node*6], pa);
        }

        if (parent != -1) {
            rd_abi_congruence_accum(&state->T_dyn[node*16], d->axis_rot,
                                    Ia, RD_IA(parent));
            rd_spatial_transform_force_accum(&state->T_dyn[node*16], pa,
                                             &pA[parent*6]);
        }
        /* For the root, Ia and pa already alias IA[node]/pA[node], which is
         * exactly what the base solve in pass 3 reads. */
    }

    /* --- Pass 3 (outward): accelerations ----------------------------------- */
    rd_real_t g_world[6];
    algo_gravity_world(gravity, g_world);

    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        const rd_dyn_node_t* d = &chain->dyn[di];
        const rd_idx_t node   = d->node;
        const rd_idx_t parent = d->danc;
        const rd_real_t* Td  = &state->T_dyn[node*16];
        const rd_int_t vi = d->vidx;

        /*
         * Acceleration inherited from the parent, in this link's frame, with
         * the velocity-product term folded in. It belongs here rather than
         * after the joint solve: expanding tau = S^T(IA a + pA) with
         * a = a' + c + S qdd gives qdd = (u - U^T(a' + c)) / D, so c has to be
         * inside the bracket.
         */
        rd_real_t a_p[6];
        if (parent == -1) {
            rd_spatial_transform_motion_inv(Td, g_world, a_p);
        } else {
            rd_spatial_transform_motion_inv(Td, &a[parent*6], a_p);
        }
        for (int k = 0; k < 6; ++k) a_p[k] += c[node*6 + k];

        if (parent == -1 && chain->has_floating_base) {
            /*
             * The base is a free joint, so solve for its acceleration:
             *   IA a_root = tau_base - pA_root
             * a_root is the gravity-loaded pseudo-acceleration that the rest of
             * the outward pass propagates; the value handed back to the caller
             * is that minus the gravity term, which is the physical
             * acceleration and is what rd_rnea() expects as qdd[0..5].
             */
            rd_real_t rhs[6], a_root[6];
            for (int k = 0; k < 6; ++k) {
                rhs[k] = (tau ? tau[k] : RD_REAL(0.0)) - pA[node*6 + k];
            }
            rd_real_t M6[36];
            rd_abi_to_mat6(RD_IA(node), M6);
            if (algo_solve6_spd(M6, rhs, a_root) != 0) {
                return RD_ERR_SINGULAR;
            }
            for (int k = 0; k < 6; ++k) {
                a[node*6 + k] = a_root[k];
                qdd_out[k]    = a_root[k] - a_p[k];
            }
            continue;
        }

        if (vi >= 0) {
            rd_real_t Ua = RD_REAL(0.0);
            for (int k = 0; k < 6; ++k) Ua += U[node*6 + k] * a_p[k];
            rd_real_t qddi = (u[node] - Ua) / D[node];
            qdd_out[vi] = qddi;
            memcpy(&a[node*6], a_p, 6*sizeof(rd_real_t));
            a[node*6 + d->s_axis] += d->s_sign * qddi;
        } else {
            memcpy(&a[node*6], a_p, 6*sizeof(rd_real_t));
        }
    }

    #undef RD_IA
    return RD_OK;
}

rd_status_t rd_aba(const rd_chain_t* chain, const rd_state_t* state,
                   const rd_real_t* tau, const rd_real_t* gravity,
                   rd_real_t* qdd_out) {
    return aba_impl(chain, state, tau, gravity, NULL, qdd_out);
}

rd_status_t rd_aba_ext(const rd_chain_t* chain, const rd_state_t* state,
                       const rd_real_t* tau, const rd_real_t* gravity,
                       const rd_real_t* f_ext, rd_real_t* qdd_out) {
    return aba_impl(chain, state, tau, gravity, f_ext, qdd_out);
}

#endif /* RD_ENABLE_ABA */

/* ============================================================================
 * Constraints: contacts, and loops the tree was cut open at
 * ============================================================================ */

rd_int_t rd_constraint_rows(const rd_constraint_t* cons, rd_int_t n_cons) {
    if (!cons) return 0;
    rd_int_t m = 0;
    for (rd_int_t i = 0; i < n_cons; ++i) {
        m += (cons[i].type == RD_CONSTRAINT_FULL) ? 6 : 3;
    }
    return m;
}

rd_int_t rd_constraint_jacobian_work(const rd_chain_t* chain) {
    return chain ? 6 * rd_chain_get_nv(chain) : 0;
}

/*
 * A spatial motion vector in the world convention is expressed at the world
 * origin; a constraint is about the point the frames meet at. Moving one there
 * leaves the angular part alone and takes p x omega off the linear part.
 */
static RD_INLINE void algo_shift_to_point(const rd_real_t p[3],
                                          const rd_real_t in[6],
                                          rd_real_t out[6]) {
    out[0] = in[0] - (p[1]*in[5] - p[2]*in[4]);
    out[1] = in[1] - (p[2]*in[3] - p[0]*in[5]);
    out[2] = in[2] - (p[0]*in[4] - p[1]*in[3]);
    out[3] = in[3]; out[4] = in[4]; out[5] = in[5];
}

/* World position of a frame, which is where its constraint rows are taken. */
static rd_status_t algo_frame_point(const rd_chain_t* chain,
                                    const rd_state_t* state,
                                    rd_idx_t frame, rd_real_t p[3]) {
    rd_real_t T[16];
    rd_status_t st = rd_forward_kinematics(chain, state, frame, T);
    if (st != RD_OK) return st;
    p[0] = T[12]; p[1] = T[13]; p[2] = T[14];
    return RD_OK;
}

rd_status_t rd_constraint_jacobian(const rd_chain_t* chain,
                                   const rd_state_t* state,
                                   const rd_constraint_t* cons, rd_int_t n_cons,
                                   rd_real_t* work,
                                   rd_real_t* J_out) {
    if (!chain || !state || !J_out) return RD_ERR_NULL_PTR;
    if (n_cons > 0 && (!cons || !work)) return RD_ERR_NULL_PTR;

    const rd_int_t nv = rd_chain_get_nv(chain);
    const rd_int_t m  = rd_constraint_rows(cons, n_cons);
    rd_zero(J_out, m * nv);

    rd_int_t row = 0;
    for (rd_int_t ci = 0; ci < n_cons; ++ci) {
        const rd_int_t nr = (cons[ci].type == RD_CONSTRAINT_FULL) ? 6 : 3;
        rd_real_t p[3];
        rd_status_t st = algo_frame_point(chain, state, cons[ci].frame_a, p);
        if (st != RD_OK) return st;

        /* frame_a adds, frame_b subtracts, both through the same scratch --
         * J_out already holds a's contribution by the time b is computed. */
        for (int side = 0; side < 2; ++side) {
            const rd_idx_t f = side ? cons[ci].frame_b : cons[ci].frame_a;
            if (side && f == RD_ANCHOR_WORLD) break;
            st = rd_jacobian(chain, state, f, RD_FRAME_WORLD, work);
            if (st != RD_OK) return st;

            for (rd_int_t k = 0; k < nv; ++k) {
                rd_real_t col[6], at_p[6];
                for (int r = 0; r < 6; ++r) col[r] = work[r*nv + k];
                algo_shift_to_point(p, col, at_p);
                for (rd_int_t r = 0; r < nr; ++r) {
                    if (side) J_out[(row + r)*nv + k] -= at_p[r];
                    else      J_out[(row + r)*nv + k] += at_p[r];
                }
            }
        }
        row += nr;
    }
    return RD_OK;
}

rd_status_t rd_constraint_bias(const rd_chain_t* chain,
                               const rd_state_t* state,
                               const rd_constraint_t* cons, rd_int_t n_cons,
                               rd_real_t* gamma_out) {
    if (!chain || !state || !gamma_out) return RD_ERR_NULL_PTR;
    if (n_cons > 0 && !cons) return RD_ERR_NULL_PTR;

    rd_zero(gamma_out, rd_constraint_rows(cons, n_cons));

    rd_int_t row = 0;
    for (rd_int_t ci = 0; ci < n_cons; ++ci) {
        const rd_int_t nr = (cons[ci].type == RD_CONSTRAINT_FULL) ? 6 : 3;
        rd_real_t p[3];
        rd_status_t st = algo_frame_point(chain, state, cons[ci].frame_a, p);
        if (st != RD_OK) return st;

        /*
         * The reference point rides on frame_a, so its velocity is frame_a's
         * and *both* sides differentiate against that one. Using each body's
         * own velocity for its own term is the obvious thing and it is wrong:
         * the two agree only while the loop is exactly closed, which is why
         * the error hides in every well-posed configuration and appears the
         * moment the constraint is violated -- exactly when a solver needs the
         * bias to be right.
         */
        rd_real_t pdot[3] = { RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0) };

        for (int side = 0; side < 2; ++side) {
            const rd_idx_t f = side ? cons[ci].frame_b : cons[ci].frame_a;
            if (side && f == RD_ANCHOR_WORLD) break;

            rd_real_t v[6], a[6], v_p[6], a_p[6];
            st = rd_spatial_velocity(chain, state, f, RD_FRAME_WORLD, v);
            if (st != RD_OK) return st;
            st = rd_spatial_acceleration(chain, state, NULL, f, RD_FRAME_WORLD, a);
            if (st != RD_OK) return st;
            algo_shift_to_point(p, v, v_p);
            algo_shift_to_point(p, a, a_p);
            if (!side) { pdot[0] = v_p[0]; pdot[1] = v_p[1]; pdot[2] = v_p[2]; }

            /* Differentiating J qd gives the *classical* acceleration of the
             * point: the shifted spatial drift plus omega x pdot. The angular
             * rows have no such term. */
            a_p[0] += v_p[4]*pdot[2] - v_p[5]*pdot[1];
            a_p[1] += v_p[5]*pdot[0] - v_p[3]*pdot[2];
            a_p[2] += v_p[3]*pdot[1] - v_p[4]*pdot[0];

            for (rd_int_t r = 0; r < nr; ++r) {
                if (side) gamma_out[row + r] -= a_p[r];
                else      gamma_out[row + r] += a_p[r];
            }
        }
        row += nr;
    }
    return RD_OK;
}

rd_int_t rd_constrained_dynamics_work(const rd_chain_t* chain,
                                      const rd_constraint_t* cons,
                                      rd_int_t n_cons) {
    if (!chain) return 0;
    const rd_int_t nv = rd_chain_get_nv(chain);
    const rd_int_t m  = rd_constraint_rows(cons, n_cons);
    /* M, h, J, X = M^-1 J^T, A = J X, dinv(M), dinv(A), rhs, lambda,
     * Jacobian scratch. */
    return nv*nv + nv + m*nv + m*nv + m*m + nv + m + m + m + 6*nv;
}

rd_status_t rd_constrained_dynamics_ext(const rd_chain_t* chain,
                                        const rd_state_t* state,
                                        const rd_real_t* tau,
                                        const rd_real_t* gravity,
                                        const rd_real_t* f_ext,
                                        const rd_constraint_t* cons, rd_int_t n_cons,
                                        rd_real_t* work,
                                        rd_real_t* qdd_out,
                                        rd_real_t* lambda_out) {
    if (!chain || !state || !qdd_out || !work) return RD_ERR_NULL_PTR;

    const rd_int_t nv = rd_chain_get_nv(chain);
    const rd_int_t m  = rd_constraint_rows(cons, n_cons);

    rd_real_t* M     = work;                 size_t off = (size_t)nv*nv;
    rd_real_t* h     = work + off;           off += (size_t)nv;
    rd_real_t* J     = work + off;           off += (size_t)m*nv;
    rd_real_t* X     = work + off;           off += (size_t)m*nv;
    rd_real_t* A     = work + off;           off += (size_t)m*m;
    rd_real_t* dinv  = work + off;           off += (size_t)nv;
    rd_real_t* dinvA = work + off;           off += (size_t)m;
    rd_real_t* rhs   = work + off;           off += (size_t)m;
    rd_real_t* lam   = work + off;           off += (size_t)m;
    rd_real_t* jw    = work + off;

    rd_status_t st = rd_crba(chain, state, M);
    if (st != RD_OK) return st;
    /* h is the bias the torque has to beat, and a force you already know
     * belongs in it: rd_rnea_ext() at qdd = 0 is exactly
     * C qd + g - sum J^T f_ext. */
    st = f_ext ? rd_rnea_ext(chain, state, NULL, gravity, f_ext, h)
               : rd_nonlinear_terms(chain, state, gravity, h);
    if (st != RD_OK) return st;
    if (m > 0) {
        st = rd_constraint_jacobian(chain, state, cons, n_cons, jw, J);
        if (st != RD_OK) return st;
        st = rd_constraint_bias(chain, state, cons, n_cons, rhs);
        if (st != RD_OK) return st;
    }

    /* h becomes the unconstrained right-hand side, then qdd_out the
     * unconstrained acceleration. M is left holding its own factor. */
    for (rd_int_t k = 0; k < nv; ++k) h[k] = (tau ? tau[k] : RD_REAL(0.0)) - h[k];
    if (rd_cholesky_factor(M, nv, dinv) != RD_OK) return RD_ERR_SINGULAR;
    rd_cholesky_solve(M, dinv, h, qdd_out, nv);
    if (m == 0) return RD_OK;

    /* X column j is M^-1 times row j of J, so A = J X is J M^-1 J^T. */
    for (rd_int_t j = 0; j < m; ++j) {
        rd_cholesky_solve(M, dinv, &J[j*nv], &X[j*nv], nv);
    }
    for (rd_int_t r = 0; r < m; ++r) {
        for (rd_int_t c = 0; c <= r; ++c) {
            rd_real_t s = RD_REAL(0.0);
            for (rd_int_t k = 0; k < nv; ++k) s += J[r*nv + k] * X[c*nv + k];
            A[r*m + c] = s;
            A[c*m + r] = s;
        }
        /* rhs = -(J qdd_free + gamma) */
        rd_real_t s = RD_REAL(0.0);
        for (rd_int_t k = 0; k < nv; ++k) s += J[r*nv + k] * qdd_out[k];
        rhs[r] = -(s + rhs[r]);
    }

    /* Redundant constraints make this singular; see the header. */
    if (rd_cholesky_factor(A, m, dinvA) != RD_OK) return RD_ERR_SINGULAR;
    rd_cholesky_solve(A, dinvA, rhs, lam, m);
    if (lambda_out) {
        for (rd_int_t r = 0; r < m; ++r) lambda_out[r] = lam[r];
    }

    for (rd_int_t k = 0; k < nv; ++k) {
        rd_real_t s = RD_REAL(0.0);
        for (rd_int_t c = 0; c < m; ++c) s += X[c*nv + k] * lam[c];
        qdd_out[k] += s;
    }
    return RD_OK;
}

rd_status_t rd_constrained_dynamics(const rd_chain_t* chain,
                                    const rd_state_t* state,
                                    const rd_real_t* tau,
                                    const rd_real_t* gravity,
                                    const rd_constraint_t* cons, rd_int_t n_cons,
                                    rd_real_t* work,
                                    rd_real_t* qdd_out,
                                    rd_real_t* lambda_out) {
    return rd_constrained_dynamics_ext(chain, state, tau, gravity, NULL,
                                       cons, n_cons, work, qdd_out, lambda_out);
}
