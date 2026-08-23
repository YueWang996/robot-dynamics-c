/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file validate_dump.c
 * @brief Dumps RobotDynamics results for one configuration, for cross-checking
 *        against a reference implementation.
 *
 *   ./validate_dump <robot> < config.txt
 *
 * Config on stdin, whitespace separated:
 *   q_base[7]   x y z qw qx qy qz     (floating-base models only)
 *   q_joints[nj]
 *   qd[nv] qdd[nv] tau[nv]            (packed; base first for a floating base)
 *
 * Output is line oriented: a tag, then values, at full precision.
 * See tools/validate.py, which drives this and compares against Pinocchio.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "robot_dynamics.h"

#include "model_simple_arm.h"
#include "model_spine.h"
#include "model_xarm7.h"
#include "model_go2.h"
#include "model_g1.h"

#define MAXNV 40
#define MAXN  48

static rd_chain_t chain;
static rd_state_t state;
static rd_real_t  state_buf[RD_STATE_BUF_FLOATS(MAXN)];

static rd_real_t q_base[7], q_joints[MAXNV];
static rd_real_t qd[MAXNV], qdd[MAXNV], tau_in[MAXNV], armature[MAXNV];
static rd_real_t f_ext[6 * MAXN], tau_ext[MAXNV], qdd_ext[MAXNV];
static rd_real_t M[MAXNV * MAXNV], J[6 * MAXNV], tau[MAXNV], qdd_fd[MAXNV];

static void rd(rd_real_t* dst, int n) {
    for (int i = 0; i < n; ++i) {
        double v;
        if (scanf("%lf", &v) != 1) { fprintf(stderr, "short input\n"); exit(2); }
        dst[i] = (rd_real_t)v;
    }
}

static void emit(const char* tag, int idx, const rd_real_t* v, int n) {
    printf("%s", tag);
    if (idx >= 0) printf(" %d", idx);
    for (int i = 0; i < n; ++i) printf(" %.17g", (double)v[i]);
    printf("\n");
}

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: validate_dump <robot>\n"); return 2; }

    const rd_model_t* model = NULL;
    if      (!strcmp(argv[1], "simple_arm")) model = simple_arm_model_get();
    else if (!strcmp(argv[1], "spine"))      model = spine_model_get();
    else if (!strcmp(argv[1], "xarm7"))      model = xarm7_model_get();
    else if (!strcmp(argv[1], "go2"))        model = go2_model_get();
    else if (!strcmp(argv[1], "g1"))         model = g1_model_get();
    else { fprintf(stderr, "unknown robot %s\n", argv[1]); return 2; }

    memset(&chain, 0, sizeof(chain));
    if (rd_chain_build(model, &chain) != RD_OK) { fprintf(stderr, "build failed\n"); return 1; }
    if (rd_state_init(&state, chain.n_nodes, state_buf, sizeof(state_buf)) != RD_OK) {
        fprintf(stderr, "state init failed\n"); return 1;
    }

    const int n  = chain.n_nodes;
    const int nj = chain.n_joints;
    const int nv = rd_chain_get_nv(&chain);
    const int fb = chain.has_floating_base;

    if (fb) rd(q_base, 7);
    rd(q_joints, nj);
    rd(qd, nv); rd(qdd, nv); rd(tau_in, nv); rd(armature, nv);
    rd(f_ext, 6 * n);

    /* Reflected rotor inertia, exercised on every run so that a mistake in it
     * cannot hide behind a zero. */
    for (int i = 0; i < nv; ++i) rd_chain_set_armature(&chain, i, armature[i]);

    printf("REAL_BYTES %d\n", (int)sizeof(rd_real_t));
    printf("SHAPE %d %d %d %d\n", n, nj, nv, fb);
    for (int i = 0; i < n; ++i) printf("NAME %d %s\n", i, chain.frame_names[i]);

    rd_update_kinematics(&chain, &state, fb ? q_base : NULL, q_joints, qd);

    for (int i = 0; i < n; ++i) {
        rd_real_t T[16];
        rd_forward_kinematics(&chain, &state, (rd_idx_t)i, T);
        emit("T", i, T, 16);
    }

    for (int i = 0; i < n; ++i) {
        rd_real_t v[6];
        rd_spatial_velocity(&chain, &state, (rd_idx_t)i, RD_FRAME_WORLD, v);
        emit("V", i, v, 6);
    }

    /* External forces, on every link including the fixed ones the chain folded
     * away -- that path is the one worth checking. */
    if (rd_rnea_ext(&chain, &state, qdd, NULL, f_ext, tau_ext) == RD_OK) {
        emit("TAUEXT", -1, tau_ext, nv);
    }
    if (rd_aba_ext(&chain, &state, tau_in, NULL, f_ext, qdd_ext) == RD_OK) {
        emit("QDDEXT", -1, qdd_ext, nv);
    }

    rd_rnea(&chain, &state, qdd, NULL, tau);
    emit("TAU", -1, tau, nv);

    if (rd_aba(&chain, &state, tau_in, NULL, qdd_fd) != RD_OK) {
        fprintf(stderr, "aba failed\n"); return 1;
    }
    emit("QDD", -1, qdd_fd, nv);

    rd_gravity(&chain, &state, NULL, tau);
    emit("G", -1, tau, nv);

    rd_nonlinear_terms(&chain, &state, NULL, tau);
    emit("NLE", -1, tau, nv);

    rd_crba(&chain, &state, M);
    for (int r = 0; r < nv; ++r) emit("M", r, &M[r * nv], nv);

    /* Deepest frame in the tree */
    rd_idx_t eef = 0;
    for (rd_int_t i = 1; i < n; ++i) {
        if (chain.parent_path_len[i] > chain.parent_path_len[eef]) eef = (rd_idx_t)i;
    }
    printf("EEF %d\n", eef);

    rd_jacobian(&chain, &state, eef, RD_FRAME_WORLD, J);
    for (int r = 0; r < 6; ++r) emit("JW", r, &J[r * nv], nv);

    rd_jacobian(&chain, &state, eef, RD_FRAME_LOCAL, J);
    for (int r = 0; r < 6; ++r) emit("JL", r, &J[r * nv], nv);

    rd_real_t acc[6];
    rd_spatial_acceleration(&chain, &state, qdd, eef, RD_FRAME_WORLD, acc);
    emit("ACC", -1, acc, 6);

    rd_chain_free(&chain);
    return 0;
}
