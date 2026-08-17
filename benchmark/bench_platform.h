/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file bench_platform.h
 * @brief Thin platform layer for the RobotDynamics benchmark.
 *
 * Provides a microsecond clock and an output function that work identically
 * on a host PC and on an RP2350 (both its Arm and its RISC-V cores), so the
 * same bench_main.c produces comparable numbers everywhere.
 *
 * The clock deliberately does *not* use a core cycle counter. RP2350's Arm
 * cores expose DWT->CYCCNT while the Hazard3 RISC-V cores expose the mcycle
 * CSR; those are different mechanisms with different read costs. The RP2350
 * timer instead ticks at a fixed 1 MHz derived from the reference clock and
 * is read the same way from either core, which keeps the Arm/RISC-V
 * comparison honest.
 */

#ifndef BENCH_PLATFORM_H
#define BENCH_PLATFORM_H

#include <stdint.h>
#include <stdio.h>

#ifdef BENCH_PICO
  #include "pico/stdlib.h"
  #include "hardware/clocks.h"

  static inline uint64_t bench_time_us(void) { return time_us_64(); }
  static inline uint32_t bench_clk_hz(void)  { return clock_get_hz(clk_sys); }

  #if defined(__riscv)
    #define BENCH_ARCH   "RISC-V (Hazard3, RV32IMAC_Zicsr_Zifencei_Zba_Zbb_Zbs_Zbkb)"
    #define BENCH_ARCH_SHORT "riscv"
  #else
    #define BENCH_ARCH   "Arm (Cortex-M33, ARMv8-M Mainline + FPU + DSP)"
    #define BENCH_ARCH_SHORT "arm"
  #endif
  #define BENCH_BOARD "Raspberry Pi Pico 2 (RP2350)"

/*
 * Porting to another MCU -- an STM32G4, say -- needs exactly two things: a
 * microsecond counter and the core clock. On a G4 that is a free-running 32-bit
 * timer (or DWT->CYCCNT divided by the clock):
 *
 *   #elif defined(BENCH_STM32)
 *     static inline uint64_t bench_time_us(void) { return __HAL_TIM_GET_COUNTER(&htim2); }
 *     static inline uint32_t bench_clk_hz(void)  { return HAL_RCC_GetSysClockFreq(); }
 *     #define BENCH_ARCH   "Arm (Cortex-M4F)"
 *     #define BENCH_ARCH_SHORT "stm32g4"
 *     #define BENCH_BOARD  "STM32G4"
 *
 * bench_main.c needs no changes. Report the results over whatever stdio the
 * board has; tools/report.py keys off the arch column, so a new architecture
 * appears as its own set of tables automatically.
 *
 * Note the G4 has a single-precision FPU like the M33, so the numbers should be
 * broadly comparable per clock -- unlike the RP2350's RISC-V cores.
 */

#else /* host build */
  #include <time.h>

  static inline uint64_t bench_time_us(void) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
  }
  static inline uint32_t bench_clk_hz(void) { return 0; }  /* unknown / scaling */

  #define BENCH_ARCH       "host"
  #define BENCH_ARCH_SHORT "host"
  #define BENCH_BOARD      "development machine"
#endif

#endif /* BENCH_PLATFORM_H */
