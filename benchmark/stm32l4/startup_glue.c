/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file startup_glue.c
 * @brief Vector table, reset handler and platform bring-up for STM32L413.
 *
 * Written by hand: there is no STM32Cube_FW_L4 package installed, and this
 * needs three peripherals rather than a HAL.
 *
 * The image is linked at 0x08010000, not at 0x08000000, and that is deliberate.
 * With the first flash word left erased the boot ROM's empty-check sends the
 * part to the system bootloader on every reset, so the board always comes back
 * as a USB DFU device no matter what this program does. The host launches the
 * program with the bootloader's Go command. There is no way to strand the
 * board, which matters when the only debug channel is the bootloader itself.
 *
 * Arriving via Go rather than via reset means the clock tree is in whatever
 * state the bootloader left it -- typically HSI48 for USB. Nothing here may
 * assume reset defaults, so both clock modes program the tree explicitly:
 *
 *   default          HSI16, 16 MHz, zero wait states.
 *   -DBENCH_L4_PLL80 80 MHz off the same HSI16, four wait states.
 *
 * The two are a cross-check on each other. Same core running the same code has
 * to take the same number of cycles per call at either clock; if the cycle
 * counts move between the runs then the PLL or the flash latency is set up
 * wrongly and the wall-clock figures are not to be believed.
 */

#include "stm32l413.h"
#include "l4_flash.h"

uint32_t SystemCoreClock = 16000000UL;

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);
extern void __libc_init_array(void);

void Reset_Handler(void);
void Default_Handler(void);

/* Set by whatever is about to do something that might fault, so the fault
 * handler can say where it died. */
volatile uint32_t g_fault_step;

void Reset_Handler(void) {
    SCB->VTOR = (uint32_t)0x08010000UL;      /* not at 0, and not where the ROM left it */

    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;

#if (__FPU_PRESENT == 1)
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));    /* CP10/CP11 full access */
#endif

    __libc_init_array();
    main();
    for (;;) { }
}

void Default_Handler(void) { for (;;) { } }

/* -nostartfiles drops crti.o/crtn.o, which normally supply these. There are no
 * static constructors in this program, so empty bodies are correct. */
void _init(void) { }
void _fini(void) { }

/* A fault must not hang the board: reset instead, so the ROM bootloader comes
 * back and whatever was printed before the fault can still be read out. */
extern void l4_out_flush(void);
extern int  _write(int fd, const char *buf, int len);
void HardFault_Handler(void) {
    static const char msg[] = "\n!FAULT step=";
    char d = (char)('0' + (g_fault_step % 10u));
    _write(1, msg, (int)sizeof(msg) - 1);
    _write(1, &d, 1);
    _write(1, "\n", 1);
    l4_out_flush();
    NVIC_SystemReset();
}

#define ALIAS __attribute__((weak, alias("Default_Handler")))
ALIAS void NMI_Handler(void);
ALIAS void MemManage_Handler(void);
ALIAS void BusFault_Handler(void);
ALIAS void UsageFault_Handler(void);
ALIAS void SVC_Handler(void);
ALIAS void DebugMon_Handler(void);
ALIAS void PendSV_Handler(void);
ALIAS void SysTick_Handler(void);

/* Only the core exceptions: this program takes no peripheral interrupts. */
__attribute__((section(".isr_vector"), used))
void (* const g_vectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler, HardFault_Handler, MemManage_Handler, BusFault_Handler,
    UsageFault_Handler, 0, 0, 0, 0,
    SVC_Handler, DebugMon_Handler, 0, PendSV_Handler, SysTick_Handler,
};

/* Normalise the clock tree to HSI16, from whatever the bootloader left behind.
 * Switch the CPU off the PLL before touching it, and only then drop the flash
 * latency -- lowering latency while still running fast reads garbage. */
static void clock_to_hsi16(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) { }

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);

    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0WS
               | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_0WS) { }

    SystemCoreClock = 16000000UL;
}

void stm32l4_clock_init(void) {
    clock_to_hsi16();

#ifdef BENCH_L4_PLL80
    /*
     * HSI16 -> PLL -> 80 MHz.
     *   VCO in  = 16 / M(2)  =   8 MHz   (must land in 4..16)
     *   VCO out =  8 * N(20) = 160 MHz   (must land in 64..344)
     *   SYSCLK  = 160 / R(2) =  80 MHz
     * Voltage range 1 is the reset default, so PWR needs no change. 80 MHz in
     * range 1 needs four wait states, set before the switch, not after.
     */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS
               | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS) { }

    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI
                 | ((2UL - 1UL) << RCC_PLLCFGR_PLLM_Pos)   /* M = 2  */
                 | (20UL << RCC_PLLCFGR_PLLN_Pos)          /* N = 20 */
                 | (0UL << RCC_PLLCFGR_PLLR_Pos)           /* R = 2  */
                 | RCC_PLLCFGR_PLLREN;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    SystemCoreClock = 80000000UL;
#endif
}

/* TIM2 is 32 bit: a 1 MHz free-running count wraps only every 4295 s.
 * APB1 runs undivided, so the timer clock is SYSCLK. */
void stm32l4_timer_init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    (void)RCC->APB1ENR1;
    TIM2->CR1 = 0;
    TIM2->PSC = (SystemCoreClock / 1000000UL) - 1UL;
    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CNT = 0;
    TIM2->CR1 = TIM_CR1_CEN;
}
