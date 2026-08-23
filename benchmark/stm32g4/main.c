/* SPDX-License-Identifier: Apache-2.0 */
#include "stm32g4xx.h"

extern void initialise_monitor_handles(void);   /* newlib semihosting */
void stm32g4_clock_170mhz(void);
void stm32g4_timer_init(void);
int  bench_run(void);                            /* bench_main.c */

#ifdef BENCH_G4_CORDIC
#include "backends/rd_cordic_stm32g4.h"
#endif

#ifdef BENCH_G4_CCM
/* Copy the library's code from its flash load address into CCM SRAM. Must run
 * before any rd_* call; nothing here touches the library before this point. */
extern unsigned _sccmram, _eccmram, _siccmram;
static void ccm_init(void) {
    unsigned *d = &_sccmram, *s = &_siccmram;
    while (d < &_eccmram) *d++ = *s++;
    __asm__ volatile ("dsb; isb" ::: "memory");
}
#endif

#ifdef BENCH_TRIG_ACCURACY
/*
 * TRIG=1: check rd_sincos against newlib's double sin and cos before timing
 * anything, so an RD_MATH_BACKEND is judged on accuracy first. Two bands: one
 * turn, where the reduction does nothing, and +/-200 radians, where it is the
 * whole answer. A backend that is fast and wrong fails here rather than in
 * somebody's robot.
 */
#include <stdio.h>
#include <math.h>
#include "rd_math.h"
static void trig_sweep(const char* tag) {
    const double PI = 3.14159265358979323846;
    double w[2][2] = {{0,0},{0,0}};
    const double lo[2] = {-PI, -200.0}, hi[2] = {PI, 200.0};
    for (int band = 0; band < 2; ++band) {
        for (int i = 0; i <= 40000; ++i) {
            double a = lo[band] + (hi[band] - lo[band]) * i / 40000.0;
            float s, c;
            rd_sincos((float)a, &s, &c);
            double es = fabs((double)s - sin((double)(float)a));
            double ec = fabs((double)c - cos((double)(float)a));
            if (es > w[band][0]) w[band][0] = es;
            if (ec > w[band][1]) w[band][1] = ec;
        }
    }
    printf("# trig %-12s [-pi,pi] sin %.3e cos %.3e | [-200,200] sin %.3e cos %.3e\n",
           tag, w[0][0], w[0][1], w[1][0], w[1][1]);
}

static void trig_accuracy(void) {
#ifdef BENCH_G4_CORDIC
    /* Walk the CORDIC's own precision field; it is 4x that many iterations
     * and about that many cycles, so this is the accuracy/speed curve. */
    char tag[16];
    for (unsigned prec = 3; prec <= 15; ++prec) {
        *(volatile uint32_t*)(RD_CORDIC_BASE + 0x00UL) = (prec << 4) | (1UL << 19);
        snprintf(tag, sizeof tag, "cordic P=%u", prec);
        trig_sweep(tag);
    }
    rd_cordic_init();                       /* back to the configured default */
#endif
    trig_sweep("default");

    float s, c;
    rd_sincos(5000.0f, &s, &c);             /* the hand-off above the cut-off */
    printf("# trig x=5000 dsin=%.3e dcos=%.3e\n",
           fabs((double)s - sin(5000.0)), fabs((double)c - cos(5000.0)));
}
#endif

int main(void) {
#ifdef BENCH_G4_CCM
    ccm_init();
#endif
    stm32g4_clock_170mhz();
    stm32g4_timer_init();
#ifdef BENCH_G4_CORDIC
    rd_cordic_init();                            /* after the clock tree */
#endif
    initialise_monitor_handles();
#ifdef BENCH_TRIG_ACCURACY
    trig_accuracy();
#endif
    return bench_run();
}
