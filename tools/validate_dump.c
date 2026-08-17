/**
 * @file validate_dump.c
 * @brief Dumps RobotDynamics results for one configuration, for cross-checking
 *        against a reference implementation.
 *
 *   ./validate_dump <robot> < config.txt
 *
 * Config on stdin, whitespace separated:
 *   q_base[7]   x y z qw qx qy qz     (floating-base models only)
 *   qd_base[6]  v w                   (root body frame)
 *   qdd_base[6] a alpha               (root body frame)
 *   q[nj] qd[nj] qdd[nj]
 *
 * Fixed-base models omit the three base blocks.
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

#define MAXNV 32

static rd_chain_t chain;
static rd_state_t state;
static rd_real_t  state_buf[40 * 66 + 16];

static rd_real_t q_base[7], qd_base[6], qdd_base[6];
static rd_real_t q[MAXNV], qd[MAXNV], qdd[MAXNV];
static rd_real_t M[MAXNV * MAXNV], J[6 * MAXNV], tau[MAXNV];

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
    else { fprintf(stderr, "unknown robot %s\n", argv[1]); return 2; }

    memset(&chain, 0, sizeof(chain));
    if (rd_chain_build(model, &chain) != RD_OK) { fprintf(stderr, "build failed\n"); return 1; }
    rd_state_init(&state, chain.n_nodes, state_buf, sizeof(state_buf));

    const int n  = chain.n_nodes;
    const int nj = chain.n_joints;
    const int nv = rd_chain_get_nv(&chain);
    const int fb = chain.has_floating_base;

    if (fb) { rd(q_base, 7); rd(qd_base, 6); rd(qdd_base, 6); }
    rd(q, nj); rd(qd, nj); rd(qdd, nj);

    printf("REAL_BYTES %d\n", (int)sizeof(rd_real_t));
    printf("SHAPE %d %d %d %d\n", n, nj, nv, fb);
    for (int i = 0; i < n; ++i) printf("NAME %d %s\n", i, chain.frame_names[i]);

    rd_update_kinematics_fb(&chain, &state,
                            fb ? q_base : NULL,
                            fb ? qd_base : NULL,
                            q, qd);

    /* World pose of every link frame, column-major 4x4 as the library stores it */
    for (int i = 0; i < n; ++i) emit("T", i, &state.T_world[i * 16], 16);

    /* Spatial velocity of every link, in world coordinates */
    for (int i = 0; i < n; ++i) {
        rd_real_t v[6];
        rd_get_spatial_velocity_cached(&chain, &state, (rd_idx_t)i, RD_FRAME_WORLD, v);
        emit("V", i, v, 6);
    }

    rd_rnea_cached(&chain, &state, fb ? qdd_base : NULL, qdd, NULL, tau);
    emit("TAU", -1, tau, nv);

    rd_crba_cached(&chain, &state, M);
    for (int r = 0; r < nv; ++r) emit("M", r, &M[r * nv], nv);

    /* Jacobian of the deepest frame, both reference frames */
    const rd_idx_t eef = (rd_idx_t)(n - 1);
    printf("EEF %d\n", eef);

    rd_jacobian_cached(&chain, &state, eef, RD_FRAME_WORLD, J);
    for (int r = 0; r < 6; ++r) emit("JW", r, &J[r * nv], nv);

    rd_jacobian_cached(&chain, &state, eef, RD_FRAME_LOCAL, J);
    for (int r = 0; r < 6; ++r) emit("JL", r, &J[r * nv], nv);

    /* Spatial acceleration of that frame, world coordinates */
    rd_real_t acc[6];
    rd_spatial_acceleration_cached(&chain, &state, fb ? qdd_base : NULL, qdd,
                                   eef, RD_FRAME_WORLD, acc);
    emit("ACC", -1, acc, 6);

    rd_chain_free(&chain);
    return 0;
}
