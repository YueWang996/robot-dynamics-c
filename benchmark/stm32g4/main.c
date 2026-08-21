/* SPDX-License-Identifier: Apache-2.0 */
#include "stm32g4xx.h"

extern void initialise_monitor_handles(void);   /* newlib semihosting */
void stm32g4_clock_170mhz(void);
void stm32g4_timer_init(void);
int  bench_run(void);                            /* bench_main.c */

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

int main(void) {
#ifdef BENCH_G4_CCM
    ccm_init();
#endif
    stm32g4_clock_170mhz();
    stm32g4_timer_init();
    initialise_monitor_handles();
    return bench_run();
}
