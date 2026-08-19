/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file probe_main.c
 * @brief Board bring-up probe. Run this before trusting a benchmark build.
 *
 * It answers the three questions that would otherwise be silent failures, and
 * it exercises the whole output path while doing so, so a successful probe also
 * proves the DFU download / Go / flash-write / read-back loop works:
 *
 *   1. How much RAM is actually contiguous from 0x20000000? STM32CubeProgrammer's
 *      device database lists 48 KB of SRAM1 for device 0x435; the L4 is
 *      documented to mirror SRAM2 immediately above it, giving 64 KB. The
 *      benchmark needs about 47 KB, so which of the two it is decides whether
 *      the build fits at all. Guessing wrong overruns into nothing and corrupts
 *      silently.
 *   2. Is the clock what the code thinks it is? A fixed loop is timed; the same
 *      loop must take five times fewer microseconds at 80 MHz than at 16 MHz.
 *   3. Do the erase and program routines work, and is the reserved region
 *      readable back over DFU?
 *
 * Linked with RAM_KB=32 so the stack sits at 0x20008000 and everything above
 * that is free to write test patterns into.
 */

#include <stdio.h>
#include "stm32l413.h"
#include "bench_platform.h"   /* bench_time_us() -- the same clock the benchmark uses */
#include "l4_flash.h"

void     stm32l4_clock_init(void);
void     stm32l4_timer_init(void);
void     l4_out_begin(void);
void     l4_out_flush(void);
void     l4_out_end(void);
uint32_t l4_out_error(void);

extern volatile uint32_t g_fault_step;

/* Written through a volatile pointer so the compiler cannot decide the
 * store-then-load pair is redundant and delete the very thing being tested. */
static int ram_rw(uint32_t addr, uint32_t pattern) {
    volatile uint32_t *p = (volatile uint32_t *)addr;
    *p = pattern;
    __DSB();
    return (*p == pattern);
}

static void ram_step(int step, const char *what, uint32_t addr, uint32_t pat) {
    g_fault_step = (uint32_t)step;
    printf("ram,%d,%s,0x%08lX,", step, what, (unsigned long)addr);
    l4_out_flush();                       /* survive a fault on the next line */
    int ok = ram_rw(addr, pat);
    printf("%s\n", ok ? "ok" : "READBACK_MISMATCH");
    l4_out_flush();
}

/* Roughly 3 cycles an iteration; the absolute number does not matter, only
 * that the same loop is timed at both clocks. */
__attribute__((noinline)) static void spin(uint32_t n) {
    __asm volatile ("1: subs %0, %0, #1\n\t bne 1b\n\t"
                    : "+r"(n) :: "cc");
}

int main(void) {
    stm32l4_clock_init();
    stm32l4_timer_init();

    /* Make bus faults precise, so g_fault_step names the access that died
     * rather than one some way downstream of it. */
    SCnSCB->ACTLR |= (1UL << 1);          /* DISDEFWBUF */
    __DSB();

    l4_out_begin();

    printf("RDL4PROBE1\n");
    printf("clk_hz,%lu\n",        (unsigned long)SystemCoreClock);
    printf("flash_size_kb,%u\n",  (unsigned)L4_FLASH_SIZE_REG);
    printf("uid,%08lX%08lX%08lX\n", (unsigned long)L4_UID_REG[0],
           (unsigned long)L4_UID_REG[1], (unsigned long)L4_UID_REG[2]);
    printf("flash_optr,0x%08lX\n", (unsigned long)FLASH->OPTR);
    printf("flash_acr,0x%08lX\n",  (unsigned long)FLASH->ACR);
    l4_out_flush();

    uint64_t t0 = bench_time_us();
    spin(1000000u);
    uint64_t t1 = bench_time_us();
    printf("spin_1e6_us,%lu\n", (unsigned long)(t1 - t0));
    l4_out_flush();

    /* Ordered so the cheap certainties come first and the one that may fault
     * comes last: everything printed before it is already in flash. */
    ram_step(1, "sram1_free",     0x20008000UL, 0xA5A5A5A5UL);
    ram_step(2, "sram1_top",      0x2000BFF8UL, 0x5A5A5A5AUL);
    ram_step(3, "sram2_direct",   0x10000000UL, 0xC0FFEE11UL);
    ram_step(4, "sram2_mirror",   0x2000C000UL, 0x11223344UL);
    ram_step(5, "sram2_mirrortop",0x2000FFF8UL, 0x55667788UL);

    /* Does 0x2000C000 really alias 0x10000000? If it does, the two banks are
     * one contiguous 64 KB block and the benchmark can have all of it. */
    g_fault_step = 6;
    *(volatile uint32_t *)0x10000000UL = 0xDEADBEEFUL;
    __DSB();
    printf("sram2_aliases_mirror,%s\n",
           (*(volatile uint32_t *)0x2000C000UL == 0xDEADBEEFUL) ? "yes" : "no");
    l4_out_flush();

    /* Expected to fault on a 64 KB part. Last, and deliberately so. */
    ram_step(7, "past_64k", 0x20010000UL, 0x0BADC0DEUL);

    printf("flash_err,0x%08lX\n", (unsigned long)l4_out_error());
    printf("END\n");
    l4_out_end();

    NVIC_SystemReset();
    for (;;) { }
}
