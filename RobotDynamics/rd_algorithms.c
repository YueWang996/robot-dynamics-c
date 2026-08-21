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

static RD_INLINE void algo_motion_transform(rd_int_t jtype, const rd_real_t axis[3],
                                            rd_real_t q, rd_real_t T[16]) {
    if (jtype == RD_JOINT_REVOLUTE) {
        if (rd_mat4_axis_rotation(axis, q, T)) return;
        rd_real_t R[9];
        rd_rot_axis_angle(axis, q, R);
        rd_real_t t0[3] = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)};
        rd_rot_to_mat4(R, t0, T);
    } else if (jtype == RD_JOINT_PRISMATIC) {
        rd_real_t t[3] = {axis[0]*q, axis[1]*q, axis[2]*q};
        rd_mat4_translate(t, T);
    } else {
        rd_mat4_identity(T);
    }
}

/* out = A * [p]x, row-major 3x3. Each column is a cross product: 18 mults. */
/* The spatial-inertia congruence moved to rd_math.h so that rd_chain_build
 * can fold fixed links with the same code that transforms them here. */
#define algo_transform_inertia_accumulate rd_spatial_inertia_congruence

static RD_INLINE void algo_joint_velocity(const rd_chain_t* chain,
                                          const rd_state_t* state,
                                          rd_idx_t node, rd_idx_t parent,
                                          rd_real_t out[6]) {
    (void)chain; (void)parent;
    const rd_real_t* S_i = &state->S[node*6];
    const rd_real_t vj = state->vj[node];
    for (int k = 0; k < 6; ++k) out[k] = S_i[k] * vj;
}

/* Gravity as a spatial motion vector in world coordinates: [-g; 0]. */
static RD_INLINE void algo_gravity_world(const rd_real_t* gravity, rd_real_t out[6]) {
    rd_real_t g[3] = {RD_REAL(0.0), RD_REAL(0.0), -RD_GRAVITY};
    if (gravity) { g[0] = gravity[0]; g[1] = gravity[1]; g[2] = gravity[2]; }
    out[0] = -g[0]; out[1] = -g[1]; out[2] = -g[2];
    out[3] = out[4] = out[5] = RD_REAL(0.0);
}

/* Velocity index of a node's joint, or -1 if it has none. */
/*
 * rd_update_kinematics() refreshes only the nodes that can move. A fixed
 * link's pose and velocity in its nearest moving ancestor are constants, so
 * resolving one here is a single transform -- cheaper than walking every fixed
 * link on every tick when most of them are never queried.
 */
static RD_INLINE void algo_frame_world(const rd_chain_t* chain,
                                       const rd_state_t* state,
                                       rd_idx_t frame, rd_real_t T_out[16]) {
    if (rd_chain_node_is_dynamic(chain, frame)) {
        memcpy(T_out, &state->T_world[frame*16], 16*sizeof(rd_real_t));
    } else {
        rd_mat4_mul_se3(&state->T_world[chain->dyn_parent[frame]*16],
                        &state->T_dyn[frame*16], T_out);
    }
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

/* In-place Cholesky solve of a 6x6 SPD system, A x = b. Returns 0 on success. */
static int algo_solve6_spd(const rd_real_t A[36], const rd_real_t b[6], rd_real_t x[6]) {
    rd_real_t L[36];
    memcpy(L, A, 36*sizeof(rd_real_t));

    for (int j = 0; j < 6; ++j) {
        rd_real_t d = L[j*6 + j];
        for (int k = 0; k < j; ++k) d -= L[j*6 + k] * L[j*6 + k];
        if (d <= RD_EPS) return -1;               /* not positive definite */
        d = rd_sqrt(d);
        L[j*6 + j] = d;
        rd_real_t inv = RD_REAL(1.0) / d;
        for (int i = j + 1; i < 6; ++i) {
            rd_real_t s = L[i*6 + j];
            for (int k = 0; k < j; ++k) s -= L[i*6 + k] * L[j*6 + k];
            L[i*6 + j] = s * inv;
        }
    }

    /* forward substitution: L y = b */
    rd_real_t y[6];
    for (int i = 0; i < 6; ++i) {
        rd_real_t s = b[i];
        for (int k = 0; k < i; ++k) s -= L[i*6 + k] * y[k];
        y[i] = s / L[i*6 + i];
    }
    /* back substitution: L^T x = y */
    for (int i = 5; i >= 0; --i) {
        rd_real_t s = y[i];
        for (int k = i + 1; k < 6; ++k) s -= L[k*6 + i] * x[k];
        x[i] = s / L[i*6 + i];
    }
    return 0;
}

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
    rd_real_t Ttmp1[16], Ttmp2[16];

    for (rd_int_t pi = 0; pi < plen; ++pi) {
        rd_idx_t node = path[pi];
        rd_mat4_mul_se3(T_out, &chain->T_joint_offset[node*16], Ttmp1);

        rd_idx_t jidx = chain->joint_idx[node];
        if (jidx >= 0 && q_joints) {
            rd_real_t Tm[16];
            algo_motion_transform(chain->joint_type[node], &chain->axes[jidx*3],
                                  q_joints[jidx], Tm);
            rd_mat4_mul_se3(Ttmp1, Tm, Ttmp2);
        } else {
            memcpy(Ttmp2, Ttmp1, 16*sizeof(rd_real_t));
        }
        rd_mat4_mul_se3(Ttmp2, &chain->T_link_offset[node*16], T_out);
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
    memset(J_out, 0, 6 * (size_t)nv * sizeof(rd_real_t));

    /* Base columns: a unit twist in the root body frame, mapped to the world. */
    if (chain->has_floating_base) {
        const rd_real_t* T_base = &state->T_world[0];
        rd_real_t unit[6], col[6];
        for (int k = 0; k < 6; ++k) {
            memset(unit, 0, 6*sizeof(rd_real_t));
            unit[k] = RD_REAL(1.0);
            rd_spatial_transform_motion(T_base, unit, col);
            for (int r = 0; r < 6; ++r) J_out[r*nv + k] = col[r];
        }
    }

    rd_int_t plen = chain->parent_path_len[frame_id];
    const rd_idx_t* path = &chain->parent_path[frame_id * chain->n_nodes];

    for (rd_int_t pi = 0; pi < plen; ++pi) {
        rd_idx_t node = path[pi];
        rd_int_t col_idx = algo_vel_index(chain, node);
        if (col_idx < 0) continue;

        const rd_real_t* axis = &chain->axes[chain->joint_idx[node]*3];
        rd_real_t twist[6] = {0};
        if (chain->joint_type[node] == RD_JOINT_REVOLUTE) {
            twist[3] = axis[0]; twist[4] = axis[1]; twist[5] = axis[2];
        } else {
            twist[0] = axis[0]; twist[1] = axis[1]; twist[2] = axis[2];
        }

        rd_real_t col[6];
        rd_spatial_transform_motion(&state->T_world[node*16], twist, col);
        for (int r = 0; r < 6; ++r) J_out[r*nv + col_idx] = col[r];
    }

    if (ref_frame == RD_FRAME_LOCAL) {
        rd_real_t Tw[16], T_f_w[16];
        algo_frame_world(chain, state, frame_id, Tw);
        rd_mat4_inv(Tw, T_f_w);
        rd_real_t in[6], out[6];
        for (rd_int_t c = 0; c < nv; ++c) {
            for (int r = 0; r < 6; ++r) in[r] = J_out[r*nv + c];
            rd_spatial_transform_motion(T_f_w, in, out);
            for (int r = 0; r < 6; ++r) J_out[r*nv + c] = out[r];
        }
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
        rd_real_t Tw[16];
        algo_frame_world(chain, state, frame_id, Tw);
        rd_spatial_transform_motion(Tw, v_local, v_out);
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
        rd_idx_t node   = chain->dyn_order[di];
        rd_idx_t parent = chain->dyn_parent[node];
        const rd_real_t* Td  = &state->T_dyn[node*16];
        const rd_real_t* v_i = &state->v[node*6];
        const rd_real_t* S_i = &state->S[node*6];

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

        rd_int_t vi = algo_vel_index(chain, node);
        rd_real_t a_joint = (vi >= 0 && qdd) ? qdd[vi] : RD_REAL(0.0);

        rd_real_t vj[6], cor[6];
        algo_joint_velocity(chain, state, node, parent, vj);
        rd_spatial_cross_motion(v_i, vj, cor);

        for (int k = 0; k < 6; ++k) {
            a[node*6 + k] = a_p[k] + S_i[k]*a_joint + cor[k];
        }
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
        rd_real_t Tw[16];
        algo_frame_world(chain, state, frame_id, Tw);
        rd_spatial_transform_motion(Tw, a_f, a_out);
    } else {
        memcpy(a_out, a_f, 6*sizeof(rd_real_t));
    }
    return RD_OK;
}

/* ============================================================================
 * Inverse dynamics
 * ============================================================================ */

/*
 * Shared RNEA body. With use_velocity = 0 the cached link velocities are
 * treated as zero, which drops the Coriolis and bias terms and leaves
 * tau = M(q) qdd + g(q) -- that is how rd_gravity() gets g(q) without needing
 * a second rd_update_kinematics() at qd = 0.
 */
static rd_status_t rnea_impl(const rd_chain_t* chain,
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

    memset(tau_out, 0, (size_t)nv * sizeof(rd_real_t));

    /* Outward: accelerations. Over the dynamics tree, so fixed links -- whose
     * inertia rd_chain_build already folded into the moving node above them --
     * are never visited. */
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node   = chain->dyn_order[di];
        rd_idx_t parent = chain->dyn_parent[node];
        const rd_real_t* Td  = &state->T_dyn[node*16];
        const rd_real_t* S_i = &state->S[node*6];

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

        rd_int_t vi = algo_vel_index(chain, node);
        rd_real_t a_joint = (vi >= 0 && qdd) ? qdd[vi] : RD_REAL(0.0);

        rd_real_t cor[6];
        if (use_velocity) {
            rd_real_t vj[6];
            algo_joint_velocity(chain, state, node, parent, vj);
            rd_spatial_cross_motion(&state->v[node*6], vj, cor);
        } else {
            memset(cor, 0, 6*sizeof(rd_real_t));
        }

        for (int k = 0; k < 6; ++k) {
            a[node*6 + k] = a_p[k] + S_i[k]*a_joint + cor[k];
        }
    }

    /* Inward: forces */
    for (rd_int_t di = chain->n_dyn - 1; di >= 0; --di) {
        rd_idx_t node = chain->dyn_order[di];
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
            rd_real_t f_child[6];
            rd_spatial_transform_force(&state->T_dyn[child*16],
                                       &f[child*6], f_child);
            for (int k = 0; k < 6; ++k) f_i[k] += f_child[k];
        }
    }

    /* Project onto the joint axes */
    rd_idx_t root = chain->topo_order[0];
    if (chain->has_floating_base) {
        for (int k = 0; k < 6; ++k) tau_out[k] = f[root*6 + k];
    }
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node = chain->dyn_order[di];
        rd_int_t vi = algo_vel_index(chain, node);
        if (vi < 0) continue;
        rd_real_t val = RD_REAL(0.0);
        for (int k = 0; k < 6; ++k) val += state->S[node*6 + k] * f[node*6 + k];
        tau_out[vi] = val;
    }
    return RD_OK;
}

rd_status_t rd_rnea(const rd_chain_t* chain, const rd_state_t* state,
                    const rd_real_t* qdd, const rd_real_t* gravity,
                    rd_real_t* tau_out) {
    return rnea_impl(chain, state, qdd, gravity, 1, tau_out);
}

rd_status_t rd_gravity(const rd_chain_t* chain, const rd_state_t* state,
                       const rd_real_t* gravity, rd_real_t* tau_out) {
    return rnea_impl(chain, state, NULL, gravity, 0, tau_out);
}

rd_status_t rd_nonlinear_terms(const rd_chain_t* chain, const rd_state_t* state,
                               const rd_real_t* gravity, rd_real_t* tau_out) {
    return rnea_impl(chain, state, NULL, gravity, 1, tau_out);
}

rd_status_t rd_coriolis(const rd_chain_t* chain, const rd_state_t* state,
                        rd_real_t* tau_out) {
    rd_real_t zero_g[3] = {RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)};
    return rnea_impl(chain, state, NULL, zero_g, 1, tau_out);
}

/* ============================================================================
 * Mass matrix
 * ============================================================================ */

rd_status_t rd_crba(const rd_chain_t* chain, const rd_state_t* state,
                    rd_real_t* M_out) {
    if (!chain || !state || !M_out) return RD_ERR_NULL_PTR;

    const rd_int_t nv = rd_chain_get_nv(chain);
    rd_real_t* Ic = state->inertia;

    memset(M_out, 0, (size_t)nv * nv * sizeof(rd_real_t));

    /* Only the dynamics nodes carry inertia after the fold, so only they need
     * seeding -- on Go2 that is 13 copies of 36 floats instead of 31. */
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node = chain->dyn_order[di];
        memcpy(&Ic[node*36], &chain->spatial_inertias[node*36],
               36 * sizeof(rd_real_t));
    }

    /* Inward: composite inertia */
    for (rd_int_t di = chain->n_dyn - 1; di >= 0; --di) {
        rd_idx_t node   = chain->dyn_order[di];
        rd_idx_t parent = chain->dyn_parent[node];
        if (parent != -1) {
            rd_real_t Ti[16];
            rd_mat4_inv(&state->T_dyn[node*16], Ti);
            algo_transform_inertia_accumulate(Ti, &Ic[node*36], &Ic[parent*36]);
        }
    }

    rd_idx_t root = chain->topo_order[0];
    if (chain->has_floating_base) {
        const rd_real_t* Ic_root = &Ic[root*36];
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                M_out[r*nv + c] = Ic_root[r*6 + c];
    }

    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node = chain->dyn_order[di];
        rd_int_t col = algo_vel_index(chain, node);
        if (col < 0) continue;

        const rd_real_t* S_i = &state->S[node*6];
        rd_real_t F[6];
        rd_mat6_vec(&Ic[node*36], S_i, F);

        rd_real_t diag = RD_REAL(0.0);
        for (int k = 0; k < 6; ++k) diag += S_i[k] * F[k];
        M_out[col*nv + col] = diag;

        /*
         * Walk the parent chain once, filling both the joint-joint couplings
         * and, on reaching the root, the floating-base coupling. The previous
         * version walked it twice for floating-base models.
         */
        rd_real_t f_prop[6];
        memcpy(f_prop, F, 6*sizeof(rd_real_t));
        rd_idx_t curr = node;
        for (;;) {
            rd_idx_t par = chain->dyn_parent[curr];
            if (par == -1) break;

            rd_real_t f_new[6];
            rd_spatial_transform_force(&state->T_dyn[curr*16], f_prop, f_new);
            memcpy(f_prop, f_new, 6*sizeof(rd_real_t));
            curr = par;

            rd_int_t par_col = algo_vel_index(chain, curr);
            if (par_col >= 0) {
                rd_real_t val = RD_REAL(0.0);
                for (int k = 0; k < 6; ++k) val += state->S[curr*6 + k] * f_prop[k];
                M_out[col*nv + par_col] = val;
                M_out[par_col*nv + col] = val;
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
    return RD_OK;
}

/* ============================================================================
 * Forward dynamics (Articulated Body Algorithm)
 * ============================================================================ */

rd_status_t rd_aba(const rd_chain_t* chain, const rd_state_t* state,
                   const rd_real_t* tau, const rd_real_t* gravity,
                   rd_real_t* qdd_out) {
    if (!chain || !state || !qdd_out) return RD_ERR_NULL_PTR;

    const rd_int_t nv = rd_chain_get_nv(chain);

    rd_real_t* IA = state->inertia;   /* articulated body inertia, 36*n */
    rd_real_t* pA = state->force;     /* bias force,               6*n  */
    rd_real_t* c  = state->cvp;       /* velocity-product accel,   6*n  */
    rd_real_t* U  = state->U;
    rd_real_t* D  = state->D;
    rd_real_t* u  = state->u;
    rd_real_t* a  = state->accel;

    memset(qdd_out, 0, (size_t)nv * sizeof(rd_real_t));

    /* --- Pass 1 (outward): bias forces and velocity-product accelerations -- */
    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node   = chain->dyn_order[di];
        rd_idx_t parent = chain->dyn_parent[node];
        const rd_real_t* v_i = &state->v[node*6];

        memcpy(&IA[node*36], &chain->spatial_inertias[node*36],
               36 * sizeof(rd_real_t));

        rd_real_t vj[6];
        algo_joint_velocity(chain, state, node, parent, vj);
        rd_spatial_cross_motion(v_i, vj, &c[node*6]);

        rd_real_t Iv[6];
        rd_spatial_inertia_mul(&chain->inertia_compact[node*RD_INERTIA_COMPACT_LEN],
                               v_i, Iv);
        rd_spatial_cross_force(v_i, Iv, &pA[node*6]);
    }

    /* --- Pass 2 (inward): articulate the inertias -------------------------- */
    for (rd_int_t di = chain->n_dyn - 1; di >= 0; --di) {
        rd_idx_t node   = chain->dyn_order[di];
        rd_idx_t parent = chain->dyn_parent[node];
        const rd_real_t* S_i = &state->S[node*6];
        rd_int_t vi = algo_vel_index(chain, node);

        /*
         * The rank-1 downdate happens in place: IA[node] is dead once this
         * link's contribution has been pushed to its parent, so copying it
         * aside first would just be 36 floats of pointless traffic.
         */
        rd_real_t* Ia = &IA[node*36];
        rd_real_t* pa = &pA[node*6];

        if (vi >= 0) {
            rd_mat6_vec(Ia, S_i, &U[node*6]);
            rd_real_t d = RD_REAL(0.0);
            for (int k = 0; k < 6; ++k) d += S_i[k] * U[node*6 + k];
            if (d <= RD_EPS) return RD_ERR_SINGULAR;
            D[node] = d;
            u[node] = (tau ? tau[vi] : RD_REAL(0.0));
            for (int k = 0; k < 6; ++k) u[node] -= S_i[k] * pa[k];

            /* Ia -= U D^-1 U^T */
            rd_real_t inv_d = RD_REAL(1.0) / d;
            for (int r = 0; r < 6; ++r) {
                rd_real_t ur = U[node*6 + r] * inv_d;
                for (int cc = 0; cc < 6; ++cc) {
                    Ia[r*6 + cc] -= ur * U[node*6 + cc];
                }
            }
            /* pa += Ia c + U D^-1 u */
            rd_real_t Iac[6];
            rd_mat6_vec(Ia, &c[node*6], Iac);
            rd_real_t ud = u[node] * inv_d;
            for (int k = 0; k < 6; ++k) pa[k] += Iac[k] + U[node*6 + k] * ud;
        } else {
            D[node] = RD_REAL(0.0);
            u[node] = RD_REAL(0.0);
            /* A rigid connection: c is zero, so pa is unchanged. */
            rd_real_t Iac[6];
            rd_mat6_vec(Ia, &c[node*6], Iac);
            for (int k = 0; k < 6; ++k) pa[k] += Iac[k];
        }

        if (parent != -1) {
            rd_real_t Tinv[16];
            rd_mat4_inv(&state->T_dyn[node*16], Tinv);
            algo_transform_inertia_accumulate(Tinv, Ia, &IA[parent*36]);
            rd_real_t pf[6];
            rd_spatial_transform_force(&state->T_dyn[node*16], pa, pf);
            for (int k = 0; k < 6; ++k) pA[parent*6 + k] += pf[k];
        }
        /* For the root, Ia and pa already alias IA[node]/pA[node], which is
         * exactly what the base solve in pass 3 reads. */
    }

    /* --- Pass 3 (outward): accelerations ----------------------------------- */
    rd_real_t g_world[6];
    algo_gravity_world(gravity, g_world);

    for (rd_int_t di = 0; di < chain->n_dyn; ++di) {
        rd_idx_t node   = chain->dyn_order[di];
        rd_idx_t parent = chain->dyn_parent[node];
        const rd_real_t* Td  = &state->T_dyn[node*16];
        const rd_real_t* S_i = &state->S[node*6];
        rd_int_t vi = algo_vel_index(chain, node);

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
            if (algo_solve6_spd(&IA[node*36], rhs, a_root) != 0) {
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
            for (int k = 0; k < 6; ++k) {
                a[node*6 + k] = a_p[k] + S_i[k]*qddi;
            }
        } else {
            memcpy(&a[node*6], a_p, 6*sizeof(rd_real_t));
        }
    }

    return RD_OK;
}
