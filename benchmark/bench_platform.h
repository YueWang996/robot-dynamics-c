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

#elif defined(BENCH_STM32G4) || defined(BENCH_STM32L4)
  #if defined(BENCH_STM32G4)
    #include "stm32g4xx.h"
    #define BENCH_ARCH       "Arm (Cortex-M4, ARMv7E-M + FPv4-SP + DSP)"
    #define BENCH_ARCH_SHORT "stm32g4"
    #define BENCH_BOARD      "STM32G474 @ 170 MHz"
  #else
    #include "stm32l413.h"
    #define BENCH_ARCH       "Arm (Cortex-M4, ARMv7E-M + FPv4-SP + DSP)"
    #define BENCH_ARCH_SHORT "stm32l4"
    #define BENCH_BOARD      "STM32L413RC"
  #endif

  extern uint32_t SystemCoreClock;

  /*
   * TIM2 free-running at 1 MHz, accumulated into 64 bits so a 32-bit wrap
   * cannot corrupt a measurement. Deliberately a timer rather than DWT->CYCCNT,
   * to match how the RP2350 side is measured.
   */
  static inline uint64_t bench_time_us(void) {
      static uint32_t last = 0;
      static uint64_t acc = 0;
      uint32_t now = TIM2->CNT;
      acc += (uint64_t)(uint32_t)(now - last);
      last = now;
      return acc;
  }
  static inline uint32_t bench_clk_hz(void) { return SystemCoreClock; }

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
