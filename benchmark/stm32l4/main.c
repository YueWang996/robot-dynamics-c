/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file main.c
 * @brief STM32L413 benchmark entry point.
 *
 * bench_main.c owns the measurements and defines BENCH_NO_MAIN so that this
 * file can bring the clock, the timer and the output channel up first.
 * Results go to flash rather than to a debug channel -- see l4_flash.h.
 */
#include <stdio.h>
#include "stm32l413.h"

void     stm32l4_clock_init(void);
void     stm32l4_timer_init(void);
void     l4_out_begin(void);
void     l4_out_end(void);
uint32_t l4_out_error(void);
uint32_t l4_out_used(void);
int      bench_run(void);

int main(void) {
    stm32l4_clock_init();
    stm32l4_timer_init();
    l4_out_begin();

    bench_run();

    printf("# flash_err=0x%08lX bytes=%lu\n",
           (unsigned long)l4_out_error(), (unsigned long)l4_out_used());
    l4_out_end();

    /* Back to the ROM bootloader, which re-enumerates as DFU so the host can
     * read the results out. Nothing to press, nothing to replug. */
    NVIC_SystemReset();
    for (;;) { }
}
