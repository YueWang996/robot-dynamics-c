/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file test_main.c
 * @brief RobotDynamics smoke test.
 *
 * Exercises every public algorithm against the built-in spine model and checks
 * properties that must hold for any correct rigid-body implementation:
 *
 *   - the mass matrix is symmetric with a positive diagonal
 *   - a rotation matrix read back out of FK is orthonormal
 *   - cached FK agrees with the standalone path-only FK
 *   - rd_rnea(qdd=0) equals rd_nonlinear_terms()
 *   - rd_gravity() is NOT the same thing (it drops the velocity terms)
 *   - ABA inverts RNEA: rnea(aba(tau)) == tau
 *
 * These are self-consistency checks. For agreement with a reference
 * implementation see docs/VALIDATION.md, which cross-checks against Pinocchio.
 */

#include <stdio.h>
#include <string.h>
#include "robot_dynamics.h"
#include "spine_model.h"

#define MAXNV (6 + RD_MAX_JOINTS)

static int g_failures = 0;

static void check(int condition, const char* what) {
    printf("  [%s] %s\n", condition ? " ok " : "FAIL", what);
    if (!condition) g_failures++;
}

static void print_matrix(const char* name, const rd_real_t* M, int rows, int cols) {
    printf("%s:\n", name);
    for (int r = 0; r < rows; ++r) {
        printf("  [");
        for (int c = 0; c < cols; ++c) {
            printf("%9.4f", (double)M[r * cols + c]);
            if (c < cols - 1) printf(",");
        }
        printf("]\n");
    }
}

static void print_vec(const char* name, const rd_real_t* v, int n) {
    printf("%s: [", name);
    for (int i = 0; i < n; ++i) {
        printf("%9.4f", (double)v[i]);
        if (i < n - 1) printf(",");
    }
    printf("]\n");
}

/* Work buffers, static so this runs on a small stack too. */
static rd_real_t state_buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
static rd_real_t J[6 * MAXNV];
static rd_real_t M[MAXNV * MAXNV];
static rd_real_t tau[MAXNV], tau_b[MAXNV], qdd_fd[MAXNV], tau_rt[MAXNV];

int main(void) {
    printf("=== RobotDynamics smoke test ===\n\n");

    const rd_model_t* model = spine_model_get();
    printf("Robot : %s\n", model->name);
    printf("Links : %u   Joints: %u   DOF: %u\n\n",
           model->num_links, model->num_joints, model->total_dof);

    rd_chain_t chain;
    memset(&chain, 0, sizeof(chain));
    if (rd_chain_build(model, &chain) != RD_OK) {
        printf("rd_chain_build failed\n");
        return 1;
    }

    const int n  = (int)chain.n_nodes;
    const int nj = (int)chain.n_joints;
    const int nv = (int)rd_chain_get_nv(&chain);

    printf("Frames:\n");
    for (int i = 0; i < n; ++i) {
        printf("  [%d] %-14s parent=%-3d joint=%d\n",
               i, chain.frame_names[i], chain.parent_list[i], chain.joint_idx[i]);
    }
    printf("\n");

    rd_state_t state;
    if (rd_state_init(&state, chain.n_nodes, state_buf, sizeof(state_buf)) != RD_OK) {
        printf("rd_state_init failed\n");
        rd_chain_free(&chain);
        return 1;
    }
    printf("State workspace: %zu bytes for %d links\n\n",
           rd_state_buffer_size(chain.n_nodes), n);

    /* ---- Test configuration --------------------------------------------- */
    rd_real_t q_base[7] = {
        RD_REAL(0.1), RD_REAL(-0.2), RD_REAL(0.5),
        RD_REAL(0.9800), RD_REAL(0.1400), RD_REAL(0.0980), RD_REAL(0.0700)
    };
    rd_real_t q_joints[RD_MAX_JOINTS] = {RD_REAL(0.1), RD_REAL(-0.1), RD_REAL(0.05)};
    /* packed nv: first six are the base twist in the root body frame */
    rd_real_t qd[MAXNV]  = {RD_REAL(0.1), RD_REAL(0.0), RD_REAL(-0.1),
                            RD_REAL(0.0), RD_REAL(0.2), RD_REAL(0.0),
                            RD_REAL(1.0), RD_REAL(-0.5), RD_REAL(0.2)};
    rd_real_t qdd[MAXNV] = {RD_REAL(0.2), RD_REAL(-0.1), RD_REAL(0.3),
                            RD_REAL(0.1), RD_REAL(0.0), RD_REAL(-0.2),
                            RD_REAL(0.3), RD_REAL(0.2), RD_REAL(-0.4)};

    if (rd_update_kinematics(&chain, &state, q_base, q_joints, qd) != RD_OK) {
        printf("rd_update_kinematics failed\n");
        rd_chain_free(&chain);
        return 1;
    }

    const rd_idx_t eef = (rd_idx_t)(n - 1);

    /* ---- Forward kinematics --------------------------------------------- */
    printf("--- Forward kinematics ---\n");
    rd_real_t T[16];
    for (int i = 0; i < n; ++i) {
        rd_forward_kinematics(&chain, &state, (rd_idx_t)i, T);
        printf("  %-14s -> [%8.4f, %8.4f, %8.4f]\n",
               chain.frame_names[i], (double)T[12], (double)T[13], (double)T[14]);
    }

    rd_forward_kinematics(&chain, &state, eef, T);
    {
        rd_real_t c0 = T[0]*T[0] + T[1]*T[1] + T[2]*T[2];
        rd_real_t c1 = T[4]*T[4] + T[5]*T[5] + T[6]*T[6];
        rd_real_t c2 = T[8]*T[8] + T[9]*T[9] + T[10]*T[10];
        check(rd_fabs(c0 - RD_REAL(1.0)) < RD_REAL(1e-4) &&
              rd_fabs(c1 - RD_REAL(1.0)) < RD_REAL(1e-4) &&
              rd_fabs(c2 - RD_REAL(1.0)) < RD_REAL(1e-4),
              "FK rotation block is orthonormal");

        rd_real_t T2[16];
        rd_fk_frame(&chain, q_base, q_joints, eef, T2);
        int same = 1;
        for (int i = 0; i < 16; ++i) {
            if (rd_fabs(T[i] - T2[i]) > RD_REAL(1e-4)) same = 0;
        }
        check(same, "cached FK matches standalone rd_fk_frame()");
    }
    printf("\n");

    /* ---- Jacobian -------------------------------------------------------- */
    printf("--- Jacobian (end frame, world) ---\n");
    rd_jacobian(&chain, &state, eef, RD_FRAME_WORLD, J);
    print_matrix("J", J, 6, nv);
    printf("\n");

    /* ---- Inverse dynamics ------------------------------------------------ */
    printf("--- Inverse dynamics (RNEA) ---\n");
    rd_rnea(&chain, &state, qdd, NULL, tau);
    print_vec("tau (base)  ", tau, 6);
    print_vec("tau (joints)", &tau[6], nj);
    printf("\n");

    /* ---- Mass matrix ----------------------------------------------------- */
    printf("--- Mass matrix (CRBA) ---\n");
    rd_crba(&chain, &state, M);
    print_matrix("M", M, nv, nv);
    {
        rd_real_t scale = RD_REAL(0.0);
        for (int r = 0; r < nv; ++r) if (M[r*nv + r] > scale) scale = M[r*nv + r];
        const rd_real_t tol = scale * RD_REAL(1e-4);

        int symmetric = 1, pos_diag = 1;
        for (int r = 0; r < nv; ++r) {
            if (M[r*nv + r] <= RD_REAL(0.0)) pos_diag = 0;
            for (int c = r + 1; c < nv; ++c) {
                if (rd_fabs(M[r*nv + c] - M[c*nv + r]) > tol) symmetric = 0;
            }
        }
        check(symmetric, "mass matrix is symmetric");
        check(pos_diag,  "mass matrix has a positive diagonal");
    }
    printf("\n");

    /* ---- Gravity vs nonlinear terms -------------------------------------- */
    printf("--- Gravity and nonlinear terms ---\n");
    rd_gravity(&chain, &state, NULL, tau);
    rd_nonlinear_terms(&chain, &state, NULL, tau_b);
    print_vec("g(q)         ", &tau[6], nj);
    print_vec("C*qd + g(q)  ", &tau_b[6], nj);
    {
        rd_real_t zero_qdd[MAXNV] = {0};
        rd_real_t tau_zero[MAXNV];
        rd_rnea(&chain, &state, zero_qdd, NULL, tau_zero);

        int agree = 1, differs = 0;
        for (int i = 0; i < nv; ++i) {
            if (rd_fabs(tau_zero[i] - tau_b[i]) > RD_REAL(1e-5)) agree = 0;
            if (rd_fabs(tau[i] - tau_b[i]) > RD_REAL(1e-5)) differs = 1;
        }
        check(agree, "rnea(qdd=0) matches nonlinear_terms()");
        check(differs, "gravity() drops the velocity terms, unlike nonlinear_terms()");
    }
    printf("\n");

    /* ---- Forward dynamics ------------------------------------------------ */
    printf("--- Forward dynamics (ABA) ---\n");
    rd_rnea(&chain, &state, qdd, NULL, tau);
    rd_status_t st = rd_aba(&chain, &state, tau, NULL, qdd_fd);
    check(st == RD_OK, "rd_aba() succeeded");
    print_vec("qdd in       ", qdd, nv);
    print_vec("qdd from ABA ", qdd_fd, nv);
    {
        rd_real_t worst = RD_REAL(0.0);
        for (int i = 0; i < nv; ++i) {
            rd_real_t e = rd_fabs(qdd_fd[i] - qdd[i]);
            if (e > worst) worst = e;
        }
        printf("max |qdd_aba - qdd| = %.3e\n", (double)worst);
        check(worst < RD_REAL(1e-3), "ABA recovers the qdd that RNEA was given");

        /* And the other direction, which also exercises the base wrench path. */
        rd_rnea(&chain, &state, qdd_fd, NULL, tau_rt);
        rd_real_t worst_tau = RD_REAL(0.0);
        for (int i = 0; i < nv; ++i) {
            rd_real_t e = rd_fabs(tau_rt[i] - tau[i]);
            if (e > worst_tau) worst_tau = e;
        }
        printf("max |rnea(aba(tau)) - tau| = %.3e\n", (double)worst_tau);
        check(worst_tau < RD_REAL(1e-3), "RNEA and ABA are mutual inverses");
    }
    printf("\n");

    /* ---- Spatial quantities ---------------------------------------------- */
    printf("--- Spatial quantities (end frame, world) ---\n");
    rd_real_t a[6], v[6];
    rd_spatial_acceleration(&chain, &state, qdd, eef, RD_FRAME_WORLD, a);
    rd_spatial_velocity(&chain, &state, eef, RD_FRAME_WORLD, v);
    print_vec("a_world", a, 6);
    print_vec("v_world", v, 6);
    printf("\n");

    rd_chain_free(&chain);

    printf("=== %s (%d failure%s) ===\n",
           g_failures ? "FAILED" : "PASSED",
           g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
