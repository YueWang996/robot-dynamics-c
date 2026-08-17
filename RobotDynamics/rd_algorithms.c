/**
 * @file rd_algorithms.c
 * @brief Robot Dynamics Algorithms Implementation
 * * Unified optimized implementation of RNEA, CRBA, Jacobian, and FK.
 */

#include "rd_algorithms.h"
#include "rd_math.h"
#include <string.h>

/* ============================================================================
 * Internal Static Helpers
 * ============================================================================ */

/* Shared Joint Transform Logic */
static RD_INLINE void algo_motion_transform(rd_int_t jtype, const rd_real_t axis[3], 
                                            rd_real_t q, rd_real_t T[16]) {
    if (jtype == RD_JOINT_REVOLUTE) {
        rd_real_t R[9];
        rd_rot_axis_angle(axis, q, R);
        rd_real_t t0[3] = {0.0f, 0.0f, 0.0f};
        rd_rot_to_mat4(R, t0, T);
    } else if (jtype == RD_JOINT_PRISMATIC) {
        rd_real_t t[3] = {axis[0]*q, axis[1]*q, axis[2]*q};
        rd_mat4_translate(t, T);
    } else {
        rd_mat4_identity(T);
    }
}

/* Transform Spatial Inertia Accumulation: I_parent += Ad(Ti)^T * I_child * Ad(Ti) */
static void algo_transform_inertia_accumulate(const rd_real_t T[16], 
                                              const rd_real_t* I_in, 
                                              rd_real_t* I_accum) {
    rd_real_t X[36];
    rd_spatial_adjoint(T, X); // Build Adjoint

    rd_real_t tmp[36];
    rd_mat6_mul(I_in, X, tmp); // I * X

    rd_real_t XT[36];
    rd_mat6_transpose(X, XT); // X^T
    
    rd_real_t res[36];
    rd_mat6_mul(XT, tmp, res); // X^T * (I * X)

    rd_mat6_add(I_accum, res, I_accum); // Accumulate
}

/* ============================================================================
 * 1. Independent Kinematics
 * ============================================================================ */

rd_status_t rd_fk_frame(const rd_chain_t* chain,
                        const rd_real_t* q_base,
                        const rd_real_t* q_joints,
                        rd_idx_t frame_id,
                        rd_real_t T_out[16]) {
    if (!chain || !T_out) return RD_ERR_NULL_PTR;
    if (frame_id < 0 || frame_id >= chain->n_nodes) return RD_ERR_INVALID_INDEX;

    /* Base Transform */
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
        rd_mat4_mul(T_out, &chain->T_joint_offset[node*16], Ttmp1);
        
        rd_idx_t jidx = chain->joint_idx[node];
        if (jidx >= 0 && q_joints) {
            rd_real_t Tm[16];
            algo_motion_transform(chain->joint_type[node], &chain->axes[jidx*3], q_joints[jidx], Tm);
            rd_mat4_mul(Ttmp1, Tm, Ttmp2);
        } else {
            memcpy(Ttmp2, Ttmp1, 16*sizeof(rd_real_t));
        }
        rd_mat4_mul(Ttmp2, &chain->T_link_offset[node*16], T_out);
    }
    return RD_OK;
}

/* ============================================================================
 * 2. Cached Dynamics
 * ============================================================================ */

rd_status_t rd_rnea_cached(const rd_chain_t* chain,
                           const rd_state_t* state,
                           const rd_real_t* qdd_base,
                           const rd_real_t* qdd_joints,
                           const rd_real_t* gravity,
                           rd_real_t* tau_out) {
    if (!chain || !state || !tau_out) return RD_ERR_NULL_PTR;

    rd_int_t n = chain->n_nodes;
    rd_int_t nv = chain->has_floating_base ? (6 + chain->n_joints) : chain->n_joints;

    /* Local buffers for Accel and Force (Velocity and Transforms are Cached) */
    rd_real_t* a = (rd_real_t*)RD_CALLOC(n * 6, sizeof(rd_real_t));
    rd_real_t* f = (rd_real_t*)RD_CALLOC(n * 6, sizeof(rd_real_t));
    if (!a || !f) { RD_FREE(a); RD_FREE(f); return RD_ERR_ALLOC_FAILED; }

    /* Gravity Setup. Carried as a spatial motion vector in WORLD coordinates;
     * the root's Ti rotates it into the base body frame below. Rotating it here
     * instead would double-rotate it once the base is no longer at identity. */
    rd_real_t g[3] = {0.0f, 0.0f, -RD_GRAVITY};
    if (gravity) { g[0]=gravity[0]; g[1]=gravity[1]; g[2]=gravity[2]; }
    const rd_real_t neg_g_world[6] = {-g[0], -g[1], -g[2], 0.0f, 0.0f, 0.0f};

    memset(tau_out, 0, nv * sizeof(rd_real_t));

    /* --- Forward Pass (Accel) --- */
    for (rd_int_t ti = 0; ti < n; ++ti) {
        rd_idx_t node = chain->topo_order[ti];
        rd_idx_t parent = chain->parent_list[node];
        rd_idx_t jidx = chain->joint_idx[node];

        const rd_real_t* Ti = &state->Ti[node*16];
        const rd_real_t* v_i = &state->v[node*6];
        const rd_real_t* S_i = &state->S[node*6];
        rd_real_t* a_i = &a[node*6];

        rd_real_t a_joint = (jidx >= 0 && qdd_joints) ? qdd_joints[jidx] : 0.0f;

        /* a_i = Ad(Ti)*a_p + S*qdd + v_i x (S*qd) */
        rd_real_t a_p_trans[6];
        if (parent == -1) {
            /* Ti maps world -> root body for the root node, so this puts gravity
             * in the same frame as qdd_base, which the caller already supplies
             * in the root body frame. */
            rd_spatial_transform_motion(Ti, neg_g_world, a_p_trans);
            if (chain->has_floating_base && qdd_base) {
                for (int k = 0; k < 6; ++k) a_p_trans[k] += qdd_base[k];
            }
        } else {
            rd_spatial_transform_motion(Ti, &a[parent*6], a_p_trans);
        }

        /* Extract S*qd contribution from velocity: v_joint = v_i - Ad(Ti)*v_p */
        rd_real_t v_p_trans[6];
        if (parent != -1) rd_spatial_transform_motion(Ti, &state->v[parent*6], v_p_trans);
        else memset(v_p_trans, 0, 6*sizeof(rd_real_t));

        rd_real_t v_joint_contrib[6];
        for(int k=0; k<6; ++k) v_joint_contrib[k] = v_i[k] - v_p_trans[k];

        rd_real_t coriolis[6];
        rd_spatial_cross_motion(v_i, v_joint_contrib, coriolis);

        for(int k=0; k<6; ++k) a_i[k] = a_p_trans[k] + S_i[k]*a_joint + coriolis[k];
    }

    /* --- Backward Pass (Force) --- */
    for (rd_int_t ti = n-1; ti >= 0; --ti) {
        rd_idx_t node = chain->topo_order[ti];
        rd_real_t* f_i = &f[node*6];
        
        rd_real_t Iv[6], Ia[6];
        const rd_real_t* I_i = &chain->spatial_inertias[node*36];
        rd_mat6_vec(I_i, &state->v[node*6], Iv);
        rd_mat6_vec(I_i, &a[node*6], Ia);
        
        rd_real_t bias[6];
        rd_spatial_cross_force(&state->v[node*6], Iv, bias);

        for(int k=0; k<6; ++k) f_i[k] = Ia[k] + bias[k];

        /* Add Children Forces */
        rd_int_t n_children = chain->children_count[node];
        for (rd_int_t ci = 0; ci < n_children; ++ci) {
            rd_idx_t child = chain->children_list[node * n + ci];
            const rd_real_t* T_pc = &state->T_parent_to_child[child*16];
            rd_real_t f_child_trans[6];
            rd_spatial_transform_force(T_pc, &f[child*6], f_child_trans);
            for(int k=0; k<6; ++k) f_i[k] += f_child_trans[k];
        }
    }

    /* Output Torques */
    if (chain->has_floating_base) {
        rd_idx_t root = chain->topo_order[0]; 
        for(int k=0; k<6; ++k) tau_out[k] = f[root*6 + k];
    }
    for (rd_int_t ti = 0; ti < n; ++ti) {
        rd_idx_t node = chain->topo_order[ti];
        rd_idx_t jidx = chain->joint_idx[node];
        if (jidx >= 0 && (chain->joint_type[node] != RD_JOINT_FIXED)) {
            rd_real_t val = 0.0f;
            for(int k=0; k<6; ++k) val += state->S[node*6 + k] * f[node*6 + k];
            rd_int_t idx = chain->has_floating_base ? (6 + jidx) : jidx;
            tau_out[idx] = val;
        }
    }

    RD_FREE(a); RD_FREE(f);
    return RD_OK;
}

rd_status_t rd_crba_cached(const rd_chain_t* chain,
                           const rd_state_t* state,
                           rd_real_t* M_out) {
    if (!chain || !state || !M_out) return RD_ERR_NULL_PTR;

    rd_int_t n = chain->n_nodes;
    rd_int_t nj = chain->n_joints;
    rd_int_t nv = chain->has_floating_base ? (6 + nj) : nj;

    rd_real_t* Ic = (rd_real_t*)RD_MALLOC(n * 36 * sizeof(rd_real_t));
    if (!Ic) return RD_ERR_ALLOC_FAILED;

    memcpy(Ic, chain->spatial_inertias, n * 36 * sizeof(rd_real_t));
    memset(M_out, 0, nv * nv * sizeof(rd_real_t));

    /* Backward Pass: Inertia Accumulation */
    for (rd_int_t ti = n-1; ti >= 0; --ti) {
        rd_idx_t node = chain->topo_order[ti];
        rd_idx_t parent = chain->parent_list[node];
        if (parent != -1) {
            /* Ic_parent += (cXp)^T * Ic_child * (cXp), where cXp is the motion
             * transform from the parent frame into this node's frame -- that is
             * Ad(Ti), Ti being the pose of the parent expressed in the child.
             * state->Ti already holds exactly that, so it goes in unmodified. */
            algo_transform_inertia_accumulate(&state->Ti[node*16],
                                              &Ic[node*36], &Ic[parent*36]);
        }
    }

    /* Assemble Mass Matrix */
    rd_idx_t vel_idx[RD_MAX_LINKS];
    for (rd_int_t i = 0; i < n; ++i) {
        rd_idx_t jidx = chain->joint_idx[i];
        vel_idx[i] = (jidx >= 0 && chain->joint_type[i] != RD_JOINT_FIXED) ? 
                     (chain->has_floating_base ? (6 + jidx) : jidx) : -1;
    }

    rd_idx_t root = chain->topo_order[0];
    if (chain->has_floating_base) {
        rd_real_t* Ic_root = &Ic[root*36];
        for (int r=0; r<6; ++r) for (int c=0; c<6; ++c) M_out[r*nv + c] = Ic_root[r*6 + c];
    }

    for (rd_int_t ti = 0; ti < n; ++ti) {
        rd_idx_t node = chain->topo_order[ti];
        rd_idx_t col_idx = vel_idx[node];
        if (col_idx == -1) continue;

        const rd_real_t* S_i = &state->S[node*6];
        rd_real_t F[6];
        rd_mat6_vec(&Ic[node*36], S_i, F);

        /* Diagonal */
        rd_real_t diag = 0.0f;
        for (int k=0; k<6; ++k) diag += S_i[k] * F[k];
        M_out[col_idx*nv + col_idx] = diag;

        /* Floating Base Coupling */
        if (chain->has_floating_base) {
            rd_real_t f_prop[6];
            memcpy(f_prop, F, 6*sizeof(rd_real_t));
            rd_idx_t curr = node;
            while (curr != root) {
                rd_idx_t par = chain->parent_list[curr];
                if (par == -1) break;
                rd_real_t f_new[6];
                rd_spatial_transform_force(&state->T_parent_to_child[curr*16], f_prop, f_new);
                memcpy(f_prop, f_new, 6*sizeof(rd_real_t));
                curr = par;
            }
            for(int k=0; k<6; ++k) { M_out[k*nv + col_idx] = f_prop[k]; M_out[col_idx*nv + k] = f_prop[k]; }
        }

        /* Joint Coupling */
        rd_real_t f_prop[6];
        memcpy(f_prop, F, 6*sizeof(rd_real_t));
        rd_idx_t curr = node;
        while (1) {
            rd_idx_t par = chain->parent_list[curr];
            if (par == -1) break;
            rd_real_t f_new[6];
            rd_spatial_transform_force(&state->T_parent_to_child[curr*16], f_prop, f_new);
            memcpy(f_prop, f_new, 6*sizeof(rd_real_t));
            curr = par;
            rd_idx_t par_col = vel_idx[curr];
            if (par_col != -1) {
                rd_real_t val = 0.0f;
                for(int k=0; k<6; ++k) val += state->S[curr*6 + k] * f_prop[k];
                M_out[col_idx*nv + par_col] = val;
                M_out[par_col*nv + col_idx] = val;
            }
        }
    }
    RD_FREE(Ic);
    return RD_OK;
}

/* ============================================================================
 * 3. Spatial Acceleration & Jacobian
 * ============================================================================ */

rd_status_t rd_spatial_acceleration_cached(const rd_chain_t* chain,
                                           const rd_state_t* state,
                                           const rd_real_t* qdd_base,
                                           const rd_real_t* qdd_joints,
                                           rd_idx_t frame_id,
                                           rd_frame_t ref_frame,
                                           rd_real_t a_out[6]) {
    /* Implementation is identical to what I provided previously, reusing rd_rnea forward pass logic */
    /* For brevity, copying the core logic: */
    if (!chain || !state || !a_out) return RD_ERR_NULL_PTR;
    
    rd_int_t n = chain->n_nodes;
    rd_real_t* a = (rd_real_t*)RD_CALLOC(n * 6, sizeof(rd_real_t));
    if (!a) return RD_ERR_ALLOC_FAILED;

    for (rd_int_t ti = 0; ti < n; ++ti) {
        rd_idx_t node = chain->topo_order[ti];
        rd_idx_t parent = chain->parent_list[node];
        rd_idx_t jidx = chain->joint_idx[node];

        const rd_real_t* Ti = &state->Ti[node*16];
        const rd_real_t* v_i = &state->v[node*6];
        const rd_real_t* S_i = &state->S[node*6];

        rd_real_t a_joint = (jidx >= 0 && qdd_joints) ? qdd_joints[jidx] : 0.0f;

        /* No gravity term here: this is a true spatial acceleration, not the
         * gravity-loaded pseudo-acceleration RNEA propagates. */
        rd_real_t a_p_trans[6];
        if (parent == -1) {
            if (chain->has_floating_base && qdd_base) {
                memcpy(a_p_trans, qdd_base, 6*sizeof(rd_real_t));
            } else {
                memset(a_p_trans, 0, 6*sizeof(rd_real_t));
            }
        } else {
            rd_spatial_transform_motion(Ti, &a[parent*6], a_p_trans);
        }

        rd_real_t v_p_trans[6];
        if (parent != -1) rd_spatial_transform_motion(Ti, &state->v[parent*6], v_p_trans);
        else memset(v_p_trans, 0, 6*sizeof(rd_real_t));

        rd_real_t v_joint_contrib[6];
        for(int k=0; k<6; ++k) v_joint_contrib[k] = v_i[k] - v_p_trans[k];

        rd_real_t coriolis[6];
        rd_spatial_cross_motion(v_i, v_joint_contrib, coriolis);

        for(int k=0; k<6; ++k) a[node*6 + k] = a_p_trans[k] + S_i[k]*a_joint + coriolis[k];
    }

    /* Extract */
    if (ref_frame == RD_FRAME_WORLD) {
        rd_spatial_transform_motion(&state->T_world[frame_id*16], &a[frame_id*6], a_out);
    } else {
        memcpy(a_out, &a[frame_id*6], 6*sizeof(rd_real_t));
    }
    RD_FREE(a);
    return RD_OK;
}

rd_status_t rd_jacobian_cached(const rd_chain_t* chain,
                               const rd_state_t* state,
                               rd_idx_t frame_id,
                               rd_frame_t ref_frame,
                               rd_real_t* J_out) {
    if (!chain || !state || !J_out) return RD_ERR_NULL_PTR;
    
    rd_int_t total_dof = chain->has_floating_base ? (6 + chain->n_joints) : chain->n_joints;
    memset(J_out, 0, 6 * total_dof * sizeof(rd_real_t));

    /* Floating Base Part */
    if (chain->has_floating_base) {
        const rd_real_t* T_base = &state->T_world[0];
        rd_real_t unit[6], col[6];
        for(int k=0; k<6; ++k) {
            memset(unit, 0, 6*sizeof(rd_real_t));
            unit[k] = 1.0f; 
            rd_spatial_transform_motion(T_base, unit, col);
            for(int r=0; r<6; ++r) J_out[r*total_dof + k] = col[r];
        }
    }

    /* Joint Part */
    rd_int_t plen = chain->parent_path_len[frame_id];
    const rd_idx_t* path = &chain->parent_path[frame_id * chain->n_nodes];
    rd_int_t base_dof = chain->has_floating_base ? 6 : 0;

    for (rd_int_t pi = 0; pi < plen; ++pi) {
        rd_idx_t node = path[pi];
        rd_idx_t jidx = chain->joint_idx[node];
        rd_int_t jtype = chain->joint_type[node];

        if (jidx < 0 || (jtype != RD_JOINT_REVOLUTE && jtype != RD_JOINT_PRISMATIC)) continue;

        const rd_real_t* T_w_j = &state->T_world[node*16];
        const rd_real_t* axis = &chain->axes[jidx*3];
        rd_real_t twist[6] = {0.0f};
        if (jtype == RD_JOINT_REVOLUTE) { twist[3]=axis[0]; twist[4]=axis[1]; twist[5]=axis[2]; }
        else { twist[0]=axis[0]; twist[1]=axis[1]; twist[2]=axis[2]; }

        rd_real_t col_vec[6];
        rd_spatial_transform_motion(T_w_j, twist, col_vec);

        rd_int_t col_idx = base_dof + jidx;
        for (int r=0; r<6; ++r) J_out[r * total_dof + col_idx] = col_vec[r];
    }

    if (ref_frame == RD_FRAME_LOCAL) {
        rd_real_t T_f_w[16];
        rd_mat4_inv(&state->T_world[frame_id*16], T_f_w);
        rd_real_t col_in[6], col_out[6];
        for (rd_int_t c = 0; c < total_dof; ++c) {
            for(int r=0; r<6; ++r) col_in[r] = J_out[r*total_dof + c];
            rd_spatial_transform_motion(T_f_w, col_in, col_out);
            for(int r=0; r<6; ++r) J_out[r*total_dof + c] = col_out[r];
        }
    }
    return RD_OK;
}

rd_status_t rd_get_spatial_velocity_cached(const rd_chain_t* chain,
                                           const rd_state_t* state,
                                           rd_idx_t frame_id,
                                           rd_frame_t ref_frame,
                                           rd_real_t v_out[6]) {
    (void)chain;
    if (!state || !v_out) return RD_ERR_NULL_PTR;
    const rd_real_t* v_local = &state->v[frame_id * 6];
    if (ref_frame == RD_FRAME_LOCAL) memcpy(v_out, v_local, 6 * sizeof(rd_real_t));
    else rd_spatial_transform_motion(&state->T_world[frame_id * 16], v_local, v_out);
    return RD_OK;
}

/* ============================================================================
 * 4. Helper Wrappers
 * ============================================================================ */

rd_status_t rd_gravity_compensation(const rd_chain_t* chain, const rd_state_t* state,
                                    const rd_real_t* gravity, rd_real_t* tau_g) {
    return rd_rnea_cached(chain, state, NULL, NULL, gravity, tau_g);
}

rd_status_t rd_nonlinear_terms(const rd_chain_t* chain, const rd_state_t* state,
                               const rd_real_t* gravity, rd_real_t* tau_nle) {
    return rd_rnea_cached(chain, state, NULL, NULL, gravity, tau_nle);
}

rd_status_t rd_coriolis(const rd_chain_t* chain, const rd_state_t* state, rd_real_t* tau_c) {
    rd_real_t zero_g[3] = {0.0f, 0.0f, 0.0f};
    return rd_rnea_cached(chain, state, NULL, NULL, zero_g, tau_c);
}
