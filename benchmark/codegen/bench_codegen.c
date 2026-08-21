/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file bench_codegen.c
 * @brief RobotDynamics against Pinocchio's code-generated dynamics.
 *
 * The generated functions take a configuration and return a result, so they
 * redo the kinematics on every call. The honest comparison is therefore
 * rd_update_kinematics() plus the algorithm against one generated call -- and,
 * separately, a control tick that wants more than one quantity, where our
 * shared state is amortised and theirs is not.
 *
 * Both are float32, both -O3, same compiler, same board. Both are checked
 * against reference outputs Pinocchio computed in double precision, so
 * neither is being graded against the other.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "robot_dynamics.h"
#include "bench_platform.h"
#include "cg_data.h"

#include "models/model_spine.h"
#include "models/model_xarm7.h"
#include "models/model_go2.h"

#define MAXNV 18
#define MAXN  40

/* CasADi's calling convention. */
typedef int (*cg_fn)(const float** arg, float** res, int* iw, float* w, int mem);
int spine_rnea(const float**, float**, int*, float*, int);
int spine_aba (const float**, float**, int*, float*, int);
int spine_crba(const float**, float**, int*, float*, int);
int xarm7_rnea(const float**, float**, int*, float*, int);
int xarm7_aba (const float**, float**, int*, float*, int);
int xarm7_crba(const float**, float**, int*, float*, int);
int go2_rnea  (const float**, float**, int*, float*, int);
int go2_aba   (const float**, float**, int*, float*, int);
int go2_crba  (const float**, float**, int*, float*, int);

static rd_real_t g_buf[RD_STATE_BUF_FLOATS(MAXN)];
static rd_real_t g_out[MAXNV * MAXNV];
static rd_real_t g_work[MAXNV * MAXNV + MAXNV];
static volatile rd_real_t g_sink;

static rd_real_t worst(const rd_real_t* a, const float* b, int n) {
    rd_real_t w = 0, scale = 0;
    for (int i = 0; i < n; ++i) {
        rd_real_t e = (rd_real_t)fabs((double)a[i] - (double)b[i]);
        rd_real_t m = (rd_real_t)fabs((double)b[i]);
        if (e > w) w = e;
        if (m > scale) scale = m;
    }
    return scale > 1 ? w / scale : w;
}

/* Calibrate to at least 50 ms, then take the minimum of five runs -- the same
 * protocol bench_main.c uses, so the numbers sit beside each other. */
#define TIME_IT(iters_out, body) do {                                        \
    uint32_t n_ = 64;                                                        \
    for (;;) {                                                               \
        uint64_t t0_ = bench_time_us();                                      \
        for (uint32_t i_ = 0; i_ < n_; ++i_) { body; }                       \
        uint64_t dt_ = bench_time_us() - t0_;                                \
        if (dt_ >= 50000u || n_ >= (1u << 22)) { break; }                    \
        n_ *= 2;                                                             \
    }                                                                        \
    uint64_t best_ = ~0ull;                                                  \
    for (int r_ = 0; r_ < 5; ++r_) {                                         \
        uint64_t t0_ = bench_time_us();                                      \
        for (uint32_t i_ = 0; i_ < n_; ++i_) { body; }                       \
        uint64_t dt_ = bench_time_us() - t0_;                                \
        if (dt_ < best_) best_ = dt_;                                        \
    }                                                                        \
    (iters_out) = (double)best_ * 1000.0 / (double)n_;   /* ns per call */   \
} while (0)

static void row(const char* robot, const char* what, double ours, double theirs,
                double err_ours, double err_theirs) {
    uint32_t hz = bench_clk_hz();
    printf("%s,%s,%.1f,%.1f,%.0f,%.0f,%.2f,%.2e,%.2e\n",
           robot, what, ours, theirs,
           hz ? ours * 1e-9 * hz : 0.0, hz ? theirs * 1e-9 * hz : 0.0,
           theirs / ours, err_ours, err_theirs);
}

static void run(const char* name, const rd_model_t* model,
                const float* qbase, const float* qjoints, const float* v,
                const float* a, const float* tau_in, const float* q_cg,
                const float* tau_ref, const float* ddq_ref, const float* M_ref,
                int nv, cg_fn f_rnea, cg_fn f_aba, cg_fn f_crba)
{
    rd_chain_t chain;
    rd_state_t st;
    if (rd_chain_build(model, &chain) != RD_OK) { printf("# %s: chain\n", name); return; }
    rd_state_init(&st, chain.n_nodes, g_buf, sizeof(g_buf));

    const float* arg[3];
    float* res[1];
    float rbuf[MAXNV * MAXNV];
    double ours, theirs, eo, et;

    /* --- inverse dynamics ------------------------------------------- */
    rd_update_kinematics(&chain, &st, qbase, qjoints, v);
    rd_rnea(&chain, &st, a, NULL, g_out);
    eo = worst(g_out, tau_ref, nv);
    arg[0] = q_cg; arg[1] = v; arg[2] = a; res[0] = rbuf;
    f_rnea(arg, res, 0, 0, 0);
    et = worst((rd_real_t*)rbuf, tau_ref, nv);   /* both float32 here */

    TIME_IT(ours, { rd_update_kinematics(&chain, &st, qbase, qjoints, v);
                    rd_rnea(&chain, &st, a, NULL, g_out); g_sink += g_out[0]; });
    TIME_IT(theirs, { arg[0]=q_cg; arg[1]=v; arg[2]=a; res[0]=rbuf;
                      f_rnea(arg,res,0,0,0); g_sink += rbuf[0]; });
    row(name, "rnea", ours, theirs, eo, et);

    /* --- forward dynamics ------------------------------------------- */
    rd_aba(&chain, &st, tau_in, NULL, g_out);
    eo = worst(g_out, ddq_ref, nv);
    arg[0]=q_cg; arg[1]=v; arg[2]=tau_in; res[0]=rbuf;
    f_aba(arg,res,0,0,0);
    et = worst((rd_real_t*)rbuf, ddq_ref, nv);
    TIME_IT(ours, { rd_update_kinematics(&chain, &st, qbase, qjoints, v);
                    rd_aba(&chain, &st, tau_in, NULL, g_out); g_sink += g_out[0]; });
    TIME_IT(theirs, { arg[0]=q_cg; arg[1]=v; arg[2]=tau_in; res[0]=rbuf;
                      f_aba(arg,res,0,0,0); g_sink += rbuf[0]; });
    row(name, "aba", ours, theirs, eo, et);

    /* --- mass matrix ------------------------------------------------- */
    rd_crba(&chain, &st, g_out);
    eo = worst(g_out, M_ref, nv*nv);
    arg[0]=q_cg; res[0]=rbuf;
    f_crba(arg,res,0,0,0);
    et = worst((rd_real_t*)rbuf, M_ref, nv*nv);
    TIME_IT(ours, { rd_update_kinematics(&chain, &st, qbase, qjoints, v);
                    rd_crba(&chain, &st, g_out); g_sink += g_out[0]; });
    TIME_IT(theirs, { arg[0]=q_cg; res[0]=rbuf; f_crba(arg,res,0,0,0);
                      g_sink += rbuf[0]; });
    row(name, "crba", ours, theirs, eo, et);

    /* --- a whole-body tick: tau and M together ----------------------- */
    TIME_IT(ours, { rd_update_kinematics(&chain, &st, qbase, qjoints, v);
                    rd_rnea(&chain, &st, a, NULL, g_out); g_sink += g_out[0];
                    rd_crba(&chain, &st, g_out); g_sink += g_out[0]; });
    TIME_IT(theirs, { arg[0]=q_cg; arg[1]=v; arg[2]=a; res[0]=rbuf;
                      f_rnea(arg,res,0,0,0); g_sink += rbuf[0];
                      arg[0]=q_cg; res[0]=rbuf; f_crba(arg,res,0,0,0);
                      g_sink += rbuf[0]; });
    row(name, "rnea+crba", ours, theirs, 0.0, 0.0);

    (void)g_work;
    rd_chain_free(&chain);
}

int bench_run(void) {
    printf("\nBEGIN_CSV\n");
    printf("# board=%s\n# arch=%s\n# clk_sys_hz=%lu\n",
           BENCH_BOARD, BENCH_ARCH, (unsigned long)bench_clk_hz());
    printf("# codegen=Pinocchio via pinocchio.casadi, CasADi C, float32, -O3\n");
    printf("# err=worst relative error against Pinocchio's own double-precision result\n");
    printf("robot,algorithm,rd_ns,codegen_ns,rd_cycles,codegen_cycles,speedup,rd_err,codegen_err\n");

    run("spine", spine_model_get(), rd_spine_qbase, rd_spine_qjoints,
        cg_spine_v, cg_spine_a, cg_spine_tau_in, cg_spine_q,
        cg_spine_tau_ref, cg_spine_ddq_ref, cg_spine_M_ref, CG_SPINE_NV,
        spine_rnea, spine_aba, spine_crba);
    run("xarm7", xarm7_model_get(), 0, rd_xarm7_qjoints,
        cg_xarm7_v, cg_xarm7_a, cg_xarm7_tau_in, cg_xarm7_q,
        cg_xarm7_tau_ref, cg_xarm7_ddq_ref, cg_xarm7_M_ref, CG_XARM7_NV,
        xarm7_rnea, xarm7_aba, xarm7_crba);
    run("go2", go2_model_get(), rd_go2_qbase, rd_go2_qjoints,
        cg_go2_v, cg_go2_a, cg_go2_tau_in, cg_go2_q,
        cg_go2_tau_ref, cg_go2_ddq_ref, cg_go2_M_ref, CG_GO2_NV,
        go2_rnea, go2_aba, go2_crba);

    printf("END_CSV\n# sink=%f\n", (double)g_sink);
    return 0;
}

#ifndef BENCH_NO_MAIN
int main(void) { return bench_run(); }
#endif
