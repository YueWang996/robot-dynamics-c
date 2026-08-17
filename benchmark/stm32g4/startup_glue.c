/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file startup_glue.c
 * @brief Minimal STM32G474 bring-up for the benchmark. No HAL.
 *
 * Does three things and nothing else: clock the core to 170 MHz, start a
 * free-running 1 MHz timer for bench_time_us(), and turn on the FPU.
 */

#include "stm32g4xx.h"

uint32_t SystemCoreClock = 16000000UL;

/* CMSIS wants these; we drive the clock ourselves so they stay trivial. */
void SystemInit(void) {
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));   /* CP10/CP11 full access */
#endif
}
void SystemCoreClockUpdate(void) { }

/*
 * HSI16 -> PLL -> 170 MHz.
 *   VCO = 16 / M(4) * N(85) = 340 MHz,  R = /2  ->  170 MHz
 * 170 MHz needs voltage-scaling range 1 *boost* and 4 flash wait states.
 */
void stm32g4_clock_170mhz(void) {
#ifdef BENCH_G4_ZEROWS
    /*
     * Stay on HSI16. At 16 MHz the flash needs ZERO wait states, so this build
     * measures the core with no flash stalls at all -- the control that says how
     * much of the 170 MHz cycle count is the flash and how much is the core.
     */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0WS
               | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) { }
    SystemCoreClock = 16000000UL;
    return;
#endif
    /* Voltage scaling range 1, then boost */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | (1UL << PWR_CR1_VOS_Pos);
    while (PWR->SR2 & PWR_SR2_VOSF) { }
    PWR->CR5 &= ~PWR_CR5_R1MODE;                 /* R1MODE=0 => boost */

    /* Flash: 4 wait states, prefetch + caches on */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS
               | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS) { }

    /* HSI16 on */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    /* PLL off before reconfiguring */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }

    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI          /* src = HSI16   */
                 | ((4UL - 1UL) << RCC_PLLCFGR_PLLM_Pos)   /* M = 4  */
                 | (85UL << RCC_PLLCFGR_PLLN_Pos)          /* N = 85 */
                 | (0UL << RCC_PLLCFGR_PLLR_Pos)           /* R = 2  */
                 | RCC_PLLCFGR_PLLREN;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    /* AHB /1, APB1 /1, APB2 /1 */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2));

    /* Switch */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    SystemCoreClock = 170000000UL;
}

/*
 * TIM2 is 32 bit on this part, so a 1 MHz free-running count wraps only every
 * 4295 s. bench_time_us() still accumulates into 64 bits so a wrap cannot
 * corrupt a measurement.
 */
void stm32g4_timer_init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    (void)RCC->APB1ENR1;
    TIM2->CR1  = 0;
    TIM2->PSC  = (SystemCoreClock / 1000000UL) - 1UL;   /* -> 1 MHz */
    TIM2->ARR  = 0xFFFFFFFFUL;
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->CNT  = 0;
    TIM2->CR1  = TIM_CR1_CEN;
}
