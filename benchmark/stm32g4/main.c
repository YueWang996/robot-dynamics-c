/* SPDX-License-Identifier: Apache-2.0 */
#include "stm32g4xx.h"

extern void initialise_monitor_handles(void);   /* newlib semihosting */
void stm32g4_clock_170mhz(void);
void stm32g4_timer_init(void);
int  bench_run(void);                            /* bench_main.c */

int main(void) {
    stm32g4_clock_170mhz();
    stm32g4_timer_init();
    initialise_monitor_handles();
    return bench_run();
}
