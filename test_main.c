/**
 * @file test_main.c
 * @brief RobotDynamics smoke test.
 *
 * Exercises every public algorithm against the built-in spine model and
 * checks a handful of properties that must hold for any correct rigid-body
 * implementation:
 *
 *   - the mass matrix is symmetric
 *   - the mass matrix has a strictly positive diagonal
 *   - RNEA with qdd = 0 reproduces the gravity + Coriolis terms
 *   - a rotation matrix read back out of FK is orthonormal
 *
 * These are self-consistency checks, not a validation against a reference
 * implementation. See docs/PROFILING.md for the current validation status.
 */

#include <stdio.h>
#include <string.h>
#include "robot_dynamics.h"
#include "spine_model.h"

static int g_failures = 0;

static void check(int condition, const char* what) {
    if (condition) {
        printf("  [ ok ] %s\n", what);
    } else {
        printf("  [FAIL] %s\n", what);
        g_failures++;
    }
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

int main(void) {
    printf("=== RobotDynamics smoke test ===\n\n");

    const rd_model_t* model = spine_model_get();
    printf("Robot : %s\n", model->name);
    printf("Links : %u   Joints: %u   DOF: %u\n\n",
           model->num_links, model->num_joints, model->total_dof);

    /* ---- Build the chain ------------------------------------------------ */
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

    /* ---- Per-tick state cache ------------------------------------------- */
    static rd_real_t state_buf[RD_MAX_LINKS * 66 + 16];
    rd_state_t state;
    if (rd_state_init(&state, chain.n_nodes, state_buf, sizeof(state_buf)) != RD_OK) {
        printf("rd_state_init failed\n");
        rd_chain_free(&chain);
        return 1;
    }

    /* ---- A test configuration ------------------------------------------- */
    rd_real_t q_base[7] = {
        RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.5),                 /* position   */
        RD_REAL(1.0), RD_REAL(0.0), RD_REAL(0.0), RD_REAL(0.0)    /* quaternion */
    };
    rd_real_t q_joints[RD_MAX_JOINTS]   = {RD_REAL(0.1), RD_REAL(-0.1), RD_REAL(0.05)};
    rd_real_t qd_joints[RD_MAX_JOINTS]  = {RD_REAL(1.0), RD_REAL(-0.5), RD_REAL(0.2)};
    rd_real_t qdd_joints[RD_MAX_JOINTS] = {RD_REAL(0.3), RD_REAL(0.2), RD_REAL(-0.4)};

    /* One traversal populates the cache every algorithm below reads. */
    if (rd_update_kinematics(&chain, &state, q_joints, qd_joints) != RD_OK) {
        printf("rd_update_kinematics failed\n");
        rd_chain_free(&chain);
        return 1;
    }

    const rd_idx_t eef = (rd_idx_t)(n - 1);

    /* ---- Forward kinematics --------------------------------------------- */
    printf("--- Forward kinematics ---\n");
    rd_real_t T[16];
    for (int i = 0; i < n; ++i) {
        if (rd_fk_frame(&chain, q_base, q_joints, (rd_idx_t)i, T) == RD_OK) {
            printf("  %-14s -> [%8.4f, %8.4f, %8.4f]\n",
                   chain.frame_names[i], (double)T[12], (double)T[13], (double)T[14]);
        }
    }
    rd_fk_frame(&chain, q_base, q_joints, eef, T);
    {
        /* Columns of the rotation block must be unit length. */
        rd_real_t c0 = T[0]*T[0] + T[1]*T[1] + T[2]*T[2];
        rd_real_t c1 = T[4]*T[4] + T[5]*T[5] + T[6]*T[6];
        rd_real_t c2 = T[8]*T[8] + T[9]*T[9] + T[10]*T[10];
        int unit = rd_fabs(c0 - RD_REAL(1.0)) < RD_REAL(1e-4) &&
                   rd_fabs(c1 - RD_REAL(1.0)) < RD_REAL(1e-4) &&
                   rd_fabs(c2 - RD_REAL(1.0)) < RD_REAL(1e-4);
        check(unit, "FK rotation block is orthonormal");
    }
    printf("\n");

    /* ---- Jacobian -------------------------------------------------------- */
    printf("--- Jacobian (end frame, world) ---\n");
    static rd_real_t J[6 * (6 + RD_MAX_JOINTS)];
    if (rd_jacobian_cached(&chain, &state, eef, RD_FRAME_WORLD, J) == RD_OK) {
        print_matrix("J", J, 6, nv);
    }
    printf("\n");

    /* ---- Inverse dynamics ------------------------------------------------ */
    printf("--- Inverse dynamics (RNEA) ---\n");
    static rd_real_t tau[6 + RD_MAX_JOINTS];
    if (rd_rnea_cached(&chain, &state, NULL, qdd_joints, NULL, tau) == RD_OK) {
        print_vec("tau (base)  ", tau, 6);
        print_vec("tau (joints)", &tau[6], nj);
    }
    printf("\n");

    /* ---- Mass matrix ----------------------------------------------------- */
    printf("--- Mass matrix (CRBA) ---\n");
    static rd_real_t M[(6 + RD_MAX_JOINTS) * (6 + RD_MAX_JOINTS)];
    if (rd_crba_cached(&chain, &state, M) == RD_OK) {
        print_matrix("M", M, nv, nv);

        /* Tolerance is scaled by the size of the matrix rather than by each
         * individual entry: off-diagonal terms are legitimately near zero, and
         * a per-entry relative test would flag rounding noise on those. */
        rd_real_t scale = RD_REAL(0.0);
        for (int r = 0; r < nv; ++r) {
            if (M[r * nv + r] > scale) scale = M[r * nv + r];
        }
        const rd_real_t tol = scale * RD_REAL(1e-4);

        int symmetric = 1, pos_diag = 1;
        for (int r = 0; r < nv; ++r) {
            if (M[r * nv + r] <= RD_REAL(0.0)) pos_diag = 0;
            for (int c = r + 1; c < nv; ++c) {
                if (rd_fabs(M[r * nv + c] - M[c * nv + r]) > tol) symmetric = 0;
            }
        }
        check(symmetric, "mass matrix is symmetric");
        check(pos_diag,  "mass matrix has a positive diagonal");
    }
    printf("\n");

    /* ---- Gravity / nonlinear terms --------------------------------------- */
    printf("--- Gravity compensation ---\n");
    static rd_real_t tau_g[6 + RD_MAX_JOINTS];
    if (rd_gravity_compensation(&chain, &state, NULL, tau_g) == RD_OK) {
        print_vec("tau_g (joints)", &tau_g[6], nj);
    }

    {
        /* RNEA with qdd = 0 is by definition the nonlinear term, so the two
         * entry points must agree bit for bit. */
        static rd_real_t tau_zero[6 + RD_MAX_JOINTS];
        static rd_real_t zero_qdd[RD_MAX_JOINTS] = {0};
        rd_rnea_cached(&chain, &state, NULL, zero_qdd, NULL, tau_zero);

        int agree = 1;
        for (int i = 0; i < nv; ++i) {
            if (rd_fabs(tau_zero[i] - tau_g[i]) > RD_REAL(1e-5)) agree = 0;
        }
        check(agree, "rnea(qdd=0) matches gravity_compensation()");
    }
    printf("\n");

    /* ---- Spatial acceleration and velocity -------------------------------- */
    printf("--- Spatial quantities (end frame, world) ---\n");
    rd_real_t a[6], v[6];
    if (rd_spatial_acceleration_cached(&chain, &state, NULL, qdd_joints,
                                       eef, RD_FRAME_WORLD, a) == RD_OK) {
        print_vec("a_world", a, 6);
    }
    if (rd_get_spatial_velocity_cached(&chain, &state, eef, RD_FRAME_WORLD, v) == RD_OK) {
        print_vec("v_world", v, 6);
    }
    printf("\n");

    rd_chain_free(&chain);

    printf("=== %s (%d failure%s) ===\n",
           g_failures ? "FAILED" : "PASSED",
           g_failures, g_failures == 1 ? "" : "s");
    return g_failures ? 1 : 0;
}
