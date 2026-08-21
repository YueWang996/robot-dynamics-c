/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file bench_main.c
 * @brief RobotDynamics micro-benchmark suite.
 *
 * Measures every algorithm the library exposes, across the same four robot
 * platforms used by the reference PyTorch implementation (bard), so the two
 * sets of numbers can be read side by side.
 *
 * Methodology
 * -----------
 *  * Each algorithm is timed in a loop whose length is auto-calibrated to run
 *    for at least CALIBRATE_TARGET_US, so the 1 us timer tick never dominates.
 *  * The calibrated loop is then run REPEATS times and the *minimum* is kept.
 *    Taking the minimum rejects interference from the USB interrupt that
 *    services stdio in the background.
 *  * Every algorithm folds a value from its output into a checksum that is
 *    printed at the end, which stops the optimiser from deleting the calls.
 *  * The cached algorithms follow the library's intended usage: a single
 *    rd_update_kinematics() populates rd_state_t, then each algorithm reuses
 *    it. update_kinematics is timed separately as its own line item.
 *
 * Output is a CSV block delimited by BEGIN_CSV / END_CSV markers so it can be
 * scraped straight off the serial port.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "robot_dynamics.h"
#include "rd_algorithms.h"
#include "bench_platform.h"

#include "models/model_simple_arm.h"
#include "models/model_spine.h"
#include "models/model_xarm7.h"
#include "models/model_go2.h"

/* ========================================================================== */
/* Tunables                                                                   */
/* ========================================================================== */

#define CALIBRATE_TARGET_US   50000u   /* >= 50 ms per timed loop */
#define MAX_ITERS             (1u << 22)
#define REPEATS               5

#ifndef BENCH_MAX_NODES
#define BENCH_MAX_NODES       40
#endif
#ifndef BENCH_MAX_NV
#define BENCH_MAX_NV          32
#endif

/* ========================================================================== */
/* Work buffers (static: the RP2350 default stack is only 2 KiB)              */
/* ========================================================================== */

static rd_chain_t g_chain;
static rd_state_t g_state;
static rd_real_t  g_state_buf[RD_STATE_BUF_FLOATS(BENCH_MAX_NODES)];

static rd_real_t  g_q_base[7];
static rd_int_t   g_has_fb;
static rd_real_t  g_q[BENCH_MAX_NV];    /* nj  */
static rd_real_t  g_qd[BENCH_MAX_NV];   /* nv, packed */
static rd_real_t  g_qdd[BENCH_MAX_NV];  /* nv, packed */
static rd_real_t  g_tau_in[BENCH_MAX_NV];

static rd_real_t  g_T[16];
static rd_real_t  g_J[6 * BENCH_MAX_NV];
static rd_real_t  g_M[BENCH_MAX_NV * BENCH_MAX_NV];
static rd_real_t  g_tau[BENCH_MAX_NV];
static rd_real_t  g_qdd_out[BENCH_MAX_NV];
static rd_real_t  g_acc[6];
static rd_real_t  g_vel[6];

static volatile rd_real_t g_checksum;

/* ========================================================================== */
/* Deterministic pseudo-random configuration                                  */
/* ========================================================================== */

static uint32_t g_rng = 0x12345678u;

static rd_real_t next_real(rd_real_t lo, rd_real_t hi) {
    /* xorshift32 -- identical sequence on every platform */
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    rd_real_t u = (rd_real_t)(g_rng >> 8) / (rd_real_t)16777216.0;
    return lo + u * (hi - lo);
}

static void make_configuration(rd_int_t nj, rd_int_t nv, rd_int_t base_dof) {
    g_rng = 0x12345678u;   /* reset so every robot sees a reproducible draw */

    /* Floating base: position + unit quaternion */
    g_q_base[0] = next_real(RD_REAL(-0.2), RD_REAL(0.2));
    g_q_base[1] = next_real(RD_REAL(-0.2), RD_REAL(0.2));
    g_q_base[2] = next_real(RD_REAL(0.3), RD_REAL(0.5));
    {
        rd_real_t w = next_real(RD_REAL(0.6), RD_REAL(1.0));
        rd_real_t x = next_real(RD_REAL(-0.3), RD_REAL(0.3));
        rd_real_t y = next_real(RD_REAL(-0.3), RD_REAL(0.3));
        rd_real_t z = next_real(RD_REAL(-0.3), RD_REAL(0.3));
        rd_real_t n = rd_sqrt(w*w + x*x + y*y + z*z);
        g_q_base[3] = w / n; g_q_base[4] = x / n;
        g_q_base[5] = y / n; g_q_base[6] = z / n;
    }
    /* Deliberately away from 0 so no sin/cos fast path is taken */
    for (rd_int_t i = 0; i < nj; ++i) {
        g_q[i] = next_real(RD_REAL(-1.0), RD_REAL(1.0));
    }
    for (rd_int_t i = 0; i < nv; ++i) {
        g_qd[i]     = next_real(RD_REAL(-1.0), RD_REAL(1.0));
        g_qdd[i]    = next_real(RD_REAL(-1.0), RD_REAL(1.0));
        g_tau_in[i] = next_real(RD_REAL(-1.0), RD_REAL(1.0));
    }
    (void)base_dof;
}

/* ========================================================================== */
/* Timing harness                                                             */
/* ========================================================================== */

typedef rd_status_t (*bench_fn_t)(const rd_chain_t*, const rd_state_t*, rd_idx_t);

typedef struct {
    const char* name;
    bench_fn_t  fn;
    int         supported;
    const char* note;
} bench_case_t;

static uint64_t run_loop(bench_fn_t fn, const rd_chain_t* chain,
                         const rd_state_t* state, rd_idx_t eef, uint32_t iters) {
    uint64_t t0 = bench_time_us();
    for (uint32_t i = 0; i < iters; ++i) {
        fn(chain, state, eef);
    }
    uint64_t t1 = bench_time_us();
    return t1 - t0;
}

/* Returns nanoseconds per call. */
static double measure(bench_fn_t fn, const rd_chain_t* chain,
                      const rd_state_t* state, rd_idx_t eef, uint32_t* iters_out) {
    uint32_t iters = 32;
    uint64_t elapsed = run_loop(fn, chain, state, eef, iters);

    while (elapsed < CALIBRATE_TARGET_US && iters < MAX_ITERS) {
        iters *= 2;
        elapsed = run_loop(fn, chain, state, eef, iters);
    }

    uint64_t best = elapsed;
    for (int r = 1; r < REPEATS; ++r) {
        uint64_t e = run_loop(fn, chain, state, eef, iters);
        if (e < best) best = e;
    }

    *iters_out = iters;
    return (double)best * 1000.0 / (double)iters;
}

/* ========================================================================== */
/* The algorithms under test                                                  */
/* ========================================================================== */

static rd_status_t case_update_kinematics(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    (void)eef;
    rd_status_t st = rd_update_kinematics(c, (rd_state_t*)s, g_has_fb ? g_q_base : NULL, g_q, g_qd);
    g_checksum += s->T_dyn[0];
    return st;
}

static rd_status_t case_fk_frame(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    (void)s;
    rd_status_t st = rd_fk_frame(c, g_q_base, g_q, eef, g_T);
    g_checksum += g_T[12];
    return st;
}

static rd_status_t case_jacobian(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    rd_status_t st = rd_jacobian(c, s, eef, RD_FRAME_WORLD, g_J);
    g_checksum += g_J[0];
    return st;
}

static rd_status_t case_jacobian_local(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    rd_status_t st = rd_jacobian(c, s, eef, RD_FRAME_LOCAL, g_J);
    g_checksum += g_J[0];
    return st;
}

static rd_status_t case_rnea(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    (void)eef;
    rd_status_t st = rd_rnea(c, s, g_qdd, NULL, g_tau);
    g_checksum += g_tau[0];
    return st;
}

static rd_status_t case_aba(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    (void)eef;
    rd_status_t st = rd_aba(c, s, g_tau_in, NULL, g_qdd_out);
    g_checksum += g_qdd_out[0];
    return st;
}

static rd_status_t case_crba(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    (void)eef;
    rd_status_t st = rd_crba(c, s, g_M);
    g_checksum += g_M[0];
    return st;
}

static rd_status_t case_gravity(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    (void)eef;
    rd_status_t st = rd_gravity(c, s, NULL, g_tau);
    g_checksum += g_tau[0];
    return st;
}

static rd_status_t case_spatial_accel(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    rd_status_t st = rd_spatial_acceleration(c, s, g_qdd, eef, RD_FRAME_WORLD, g_acc);
    g_checksum += g_acc[0];
    return st;
}

static rd_status_t case_spatial_vel(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    rd_status_t st = rd_spatial_velocity(c, s, eef, RD_FRAME_WORLD, g_vel);
    g_checksum += g_vel[0];
    return st;
}

/*
 * Probe: the heap traffic that rd_rnea_cached / rd_crba_cached perform on every
 * single call. Timing it separately lets the report attribute how much of each
 * algorithm's cost is allocator overhead rather than arithmetic.
 */
/*
 * Integer-only control. The library no longer allocates anywhere, so this is
 * not measuring the library -- it exists purely as an architecture comparison
 * that touches no floating point, which isolates how much of the Arm/RISC-V gap
 * is the missing FPU rather than the core itself.
 */
static rd_status_t case_alloc_probe(const rd_chain_t* c, const rd_state_t* s, rd_idx_t eef) {
    (void)s; (void)eef;
    rd_int_t n = c->n_nodes;
    void* a = RD_CALLOC(n * 6, sizeof(rd_real_t));
    void* f = RD_CALLOC(n * 6, sizeof(rd_real_t));
    void* ic = RD_MALLOC((size_t)n * 36 * sizeof(rd_real_t));
    g_checksum += (rd_real_t)(a != NULL) + (rd_real_t)(f != NULL) + (rd_real_t)(ic != NULL);
    RD_FREE(a); RD_FREE(f); RD_FREE(ic);
    return RD_OK;
}

static const bench_case_t g_cases[] = {
    { "update_kinematics",  case_update_kinematics, 1, "shared cache pass, call once per control tick" },
    { "fk_frame",           case_fk_frame,          1, "standalone, walks root->frame path" },
    { "jacobian_world",     case_jacobian,          1, "cached" },
    { "jacobian_local",     case_jacobian_local,    1, "cached, adds a 6xnv reference-frame change" },
    { "rnea",               case_rnea,              1, "inverse dynamics" },
    { "aba",                case_aba,               1, "forward dynamics" },
    { "crba",               case_crba,              1, "cached, joint-space mass matrix" },
    { "gravity_comp",       case_gravity,           1, "cached, RNEA with qdd=0" },
    { "spatial_accel",      case_spatial_accel,     1, "cached, re-runs the forward pass" },
    { "spatial_velocity",   case_spatial_vel,       1, "cached, O(1) lookup + transform" },
    { "_heap_probe",        case_alloc_probe,       1, "PROBE: integer-only control, no float at all" },
};

#define N_CASES ((int)(sizeof(g_cases) / sizeof(g_cases[0])))

/* ========================================================================== */
/* Robot platforms                                                            */
/* ========================================================================== */

typedef struct {
    const char* name;
    const rd_model_t* (*get)(void);
    const char* origin;
} bench_robot_t;

static const bench_robot_t g_robots[] = {
    { "simple_arm", simple_arm_model_get, "bard/examples/simple_arm.urdf" },
    { "spine",      spine_model_get,      "bard/tests/spine.urdf" },
    { "xarm7",      xarm7_model_get,      "bard/examples/.../xarm7.urdf" },
    { "go2",        go2_model_get,        "bard/examples/.../go2.urdf" },
};

#define N_ROBOTS ((int)(sizeof(g_robots) / sizeof(g_robots[0])))

/* ========================================================================== */

static void run_robot(const bench_robot_t* robot) {
    const rd_model_t* model = robot->get();

    memset(&g_chain, 0, sizeof(g_chain));
    if (rd_chain_build(model, &g_chain) != RD_OK) {
        printf("ERROR,%s,chain_build_failed\n", robot->name);
        return;
    }

    rd_int_t n  = g_chain.n_nodes;
    rd_int_t nj = g_chain.n_joints;
    rd_int_t nv = rd_chain_get_nv(&g_chain);

    if (n > BENCH_MAX_NODES || nv > BENCH_MAX_NV) {
        printf("ERROR,%s,model_exceeds_bench_buffers n=%d nv=%d\n",
               robot->name, (int)n, (int)nv);
        rd_chain_free(&g_chain);
        return;
    }

    if (rd_state_init(&g_state, n, g_state_buf, sizeof(g_state_buf)) != RD_OK) {
        printf("ERROR,%s,state_init_failed\n", robot->name);
        rd_chain_free(&g_chain);
        return;
    }

    g_has_fb = g_chain.has_floating_base;
    make_configuration(nj, nv, g_has_fb ? 6 : 0);

    /* Query frame for FK/Jacobian/spatial rows: the deepest node in the tree.
     * Picked by path length rather than by index, so that reordering the model
     * cannot silently change which frame these rows describe. */
    rd_idx_t eef = 0;
    for (rd_int_t i = 1; i < n; ++i) {
        if (g_chain.parent_path_len[i] > g_chain.parent_path_len[eef]) {
            eef = (rd_idx_t)i;
        }
    }

    /* Prime the cache exactly the way a control loop would. */
    rd_update_kinematics(&g_chain, &g_state,
                         g_has_fb ? g_q_base : NULL, g_q, g_qd);

    for (int i = 0; i < N_CASES; ++i) {
        /*
         * Run once and check the status first. An algorithm that bails out
         * early -- rd_aba on a model whose links carry no inertia, for
         * instance -- would otherwise be "timed" at the cost of its error
         * return, which is worse than no number at all.
         */
        rd_status_t st = g_cases[i].fn(&g_chain, &g_state, eef);
        if (st != RD_OK) {
            printf("%s,%s,%s,%d,%d,%d,,,unsupported(%d)\n",
                   BENCH_ARCH_SHORT, robot->name, g_cases[i].name,
                   (int)n, (int)nj, (int)nv, (int)st);
            continue;
        }

        uint32_t iters = 0;
        double ns = measure(g_cases[i].fn, &g_chain, &g_state, eef, &iters);

        uint32_t hz = bench_clk_hz();
        double cycles = (hz > 0) ? (ns * 1e-9 * (double)hz) : 0.0;

        printf("%s,%s,%s,%d,%d,%d,%.1f,%.0f,%lu\n",
               BENCH_ARCH_SHORT, robot->name, g_cases[i].name,
               (int)n, (int)nj, (int)nv,
               ns, cycles, (unsigned long)iters);
    }

    rd_chain_free(&g_chain);
}

static void report(void) {
    printf("\nBEGIN_CSV\n");
    printf("# board=%s\n", BENCH_BOARD);
    printf("# arch=%s\n", BENCH_ARCH);
    printf("# clk_sys_hz=%lu\n", (unsigned long)bench_clk_hz());
    printf("# real_t=%s\n", RD_REAL_IS_FLOAT ? "float32" : "float64");
    printf("# compiler=" __VERSION__ "\n");
    printf("# repeats=%d target_us=%u (minimum of %d timed loops)\n",
           REPEATS, CALIBRATE_TARGET_US, REPEATS);
    printf("arch,robot,algorithm,n_links,n_joints,nv,ns_per_call,cycles_per_call,iters\n");

    for (int r = 0; r < N_ROBOTS; ++r) {
        run_robot(&g_robots[r]);
    }

    printf("END_CSV\n");
    printf("# checksum=%.6f (ignore; exists to defeat dead-code elimination)\n",
           (double)g_checksum);
}

/* Entry point. On STM32 the reset handler needs to bring up clocks and
 * semihosting first, so main() lives in that port and calls this instead. */
int bench_run(void) {
#ifdef BENCH_PICO
    stdio_init_all();

    /* Give the host a chance to attach; then report on a loop so a late
     * attach still catches a complete run. */
    for (int i = 0; i < 300 && !stdio_usb_connected(); ++i) {
        sleep_ms(50);
    }
    sleep_ms(500);

    while (true) {
        report();
        sleep_ms(3000);
    }
#else
    report();
    return 0;
#endif
}

/* The bare-metal ports own main(): they must bring up clocks and stdio before
 * anything here runs. They define BENCH_NO_MAIN to say so. */
#ifndef BENCH_NO_MAIN
int main(void) { return bench_run(); }
#endif
