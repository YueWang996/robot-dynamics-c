/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file trig_main.c
 * @brief What sin/cos actually costs, and what the alternatives buy.
 *
 * update_kinematics and fk_frame are the only routines in the library that call
 * libm, and they are also the two that pay most for flash wait states. That
 * says the trig code is fetch-heavy; it does not say it dominates the run time.
 * This measures the difference, on the part, so the question is settled by data
 * rather than by which explanation sounds better.
 *
 * Four ways to get sin and cos of the same angle:
 *
 *   sincosf      newlib, one reduction for both -- what rd_sincos uses now
 *   sinf + cosf  newlib, two separate calls, for reference
 *   CMSIS-DSP    arm_sin_f32/arm_cos_f32: 512-entry table + linear interpolation
 *   poly         Cody-Waite reduction over four quadrants + minimax polynomials
 *
 * Accuracy is measured against double-precision libm on the same sweep, because
 * a faster wrong answer is not an optimisation.
 */

#include <math.h>
#include <stdio.h>
#include "stm32l413.h"
#include "bench_platform.h"
#include "l4_flash.h"
#include "arm_math.h"

void     stm32l4_clock_init(void);
void     stm32l4_timer_init(void);
void     l4_out_begin(void);
void     l4_out_flush(void);
void     l4_out_end(void);

#define NANG  64
#define REPS  3125                     /* NANG * REPS = 200,000 calls */

static float g_ang[NANG];
static volatile float g_sink;

/* Cody-Waite reduction to [-pi/4, pi/4] plus a quadrant, then degree-7/6
 * minimax polynomials. Joint angles are bounded, so none of libm's general
 * Payne-Hanek machinery for huge arguments is needed. */
static inline void poly_sincos(float x, float *sn, float *cs) {
    /* Round-to-nearest without a libm call: FPv4-SP has no VRINT. */
    const float MAGIC = 12582912.0f;               /* 2^23 + 2^22 */
    float fn = (x * 0.636619772f + MAGIC) - MAGIC; /* n = round(x * 2/pi) */
    int   n  = (int)fn;
    float r  = x - fn * 1.5707962513e+00f;         /* pi/2, high part */
    r        =   r - fn * 7.5497894159e-08f;       /* pi/2, low part  */

    float r2 = r * r;
    float s  = r * (1.0f + r2 * (-1.6666667e-01f + r2 * ( 8.3333337e-03f
                     + r2 * (-1.9841270e-04f + r2 * ( 2.7557319e-06f)))));
    float c  = 1.0f + r2 * (-5.0000000e-01f + r2 * ( 4.1666668e-02f
                     + r2 * (-1.3888889e-03f + r2 * ( 2.4801587e-05f))));
    switch (n & 3) {
        case 0:  *sn =  s; *cs =  c; break;
        case 1:  *sn =  c; *cs = -s; break;
        case 2:  *sn = -s; *cs = -c; break;
        default: *sn = -c; *cs =  s; break;
    }
}

static uint32_t g_rng = 0x12345678u;
static float next_angle(void) {
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return ((float)(g_rng >> 8) / 16777216.0f) * 6.2831853f - 3.14159265f;
}

/* Cycles for one (sin, cos) pair, with the loop's own cost taken out. */
static uint32_t g_base_us;
static void report(const char *name, uint32_t us) {
    double calls = (double)NANG * (double)REPS;
    double net_us = (double)us - (double)g_base_us;
    if (net_us < 0) net_us = 0;
    double cyc = net_us * 1e-6 * (double)SystemCoreClock / calls;
    printf("%s,%lu,%.1f\n", name, (unsigned long)us, cyc);
    l4_out_flush();
}

#define TIME(name, BODY)                                                   \
    do {                                                                   \
        uint64_t t0 = bench_time_us();                                     \
        for (uint32_t r = 0; r < REPS; ++r)                                \
            for (int i = 0; i < NANG; ++i) { float x = g_ang[i]; BODY }    \
        uint64_t t1 = bench_time_us();                                     \
        report(name, (uint32_t)(t1 - t0));                                 \
    } while (0)

/* Max absolute error against double-precision libm, over a dense sweep of the
 * range a revolute joint can actually reach. */
static void accuracy(void) {
    double es_lib = 0, ec_lib = 0, es_arm = 0, ec_arm = 0, es_pol = 0, ec_pol = 0;
    for (int i = 0; i <= 2000; ++i) {
        double xd = -3.14159265358979 + 6.28318530717959 * i / 2000.0;
        float  x  = (float)xd;
        double rs = sin((double)x), rc = cos((double)x);
        float s, c;

        sincosf(x, &s, &c);
        if (fabs(s - rs) > es_lib) es_lib = fabs(s - rs);
        if (fabs(c - rc) > ec_lib) ec_lib = fabs(c - rc);

        s = arm_sin_f32(x); c = arm_cos_f32(x);
        if (fabs(s - rs) > es_arm) es_arm = fabs(s - rs);
        if (fabs(c - rc) > ec_arm) ec_arm = fabs(c - rc);

        poly_sincos(x, &s, &c);
        if (fabs(s - rs) > es_pol) es_pol = fabs(s - rs);
        if (fabs(c - rc) > ec_pol) ec_pol = fabs(c - rc);
    }
    printf("acc_sincosf,%.3e,%.3e\n", es_lib, ec_lib);
    printf("acc_cmsisdsp,%.3e,%.3e\n", es_arm, ec_arm);
    printf("acc_poly,%.3e,%.3e\n", es_pol, ec_pol);
    l4_out_flush();
}

int main(void) {
    stm32l4_clock_init();
    stm32l4_timer_init();
    l4_out_begin();

    for (int i = 0; i < NANG; ++i) g_ang[i] = next_angle();

    printf("RDL4TRIG1\n");
    printf("clk_hz,%lu\n", (unsigned long)SystemCoreClock);
    printf("what,elapsed_us,cycles_per_sincos_pair\n");
    l4_out_flush();

    /* Baseline first: everything else is reported net of it. */
    g_base_us = 0;
    { uint64_t t0 = bench_time_us();
      for (uint32_t r = 0; r < REPS; ++r)
          for (int i = 0; i < NANG; ++i) { float x = g_ang[i]; g_sink = x + x; }
      g_base_us = (uint32_t)(bench_time_us() - t0); }
    printf("loop_baseline,%lu,0.0\n", (unsigned long)g_base_us);
    l4_out_flush();

    { float s, c; TIME("sincosf",   sincosf(x, &s, &c); g_sink = s + c;); }
    { float s, c; TIME("sinf_cosf", s = sinf(x); c = cosf(x); g_sink = s + c;); }
    { float s, c; TIME("cmsis_dsp", s = arm_sin_f32(x); c = arm_cos_f32(x);
                                    g_sink = s + c;); }
    { float s, c; TIME("poly",      poly_sincos(x, &s, &c); g_sink = s + c;); }

    printf("what,max_abs_err_sin,max_abs_err_cos\n");
    accuracy();

    printf("END\n");
    l4_out_end();
    NVIC_SystemReset();
    for (;;) { }
}
