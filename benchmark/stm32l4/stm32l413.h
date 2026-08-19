/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file stm32l413.h
 * @brief Just enough STM32L4 register definition to run the benchmark.
 *
 * Hand-written because no STM32Cube_FW_L4 package is installed and this needs
 * four peripherals, not a HAL. core_cm4.h is generic ARM CMSIS-Core and comes
 * from whichever Cube package you do have.
 *
 * Everything here is verifiable on target: see the self-check in the README.
 */
#ifndef STM32L413_MIN_H
#define STM32L413_MIN_H

#include <stdint.h>

#define __CM4_REV              0x0001U
#define __MPU_PRESENT          1
#define __NVIC_PRIO_BITS       4
#define __Vendor_SysTickConfig 0
#define __FPU_PRESENT          1

/* core_cm4.h needs these; only the core exceptions matter here, since this
 * program takes no peripheral interrupts. */
typedef enum {
    NonMaskableInt_IRQn   = -14,
    HardFault_IRQn        = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn         = -11,
    UsageFault_IRQn       = -10,
    SVCall_IRQn           = -5,
    DebugMonitor_IRQn     = -4,
    PendSV_IRQn           = -2,
    SysTick_IRQn          = -1,
    WWDG_IRQn             = 0
} IRQn_Type;

#include "core_cm4.h"

#define PERIPH_BASE   0x40000000UL
#define RCC_BASE      (PERIPH_BASE + 0x21000UL)
#define FLASH_R_BASE  (PERIPH_BASE + 0x22000UL)
#define PWR_BASE      (PERIPH_BASE + 0x07000UL)
#define TIM2_BASE     (PERIPH_BASE + 0x00000UL)

typedef struct {
    volatile uint32_t CR;          /* 0x00 */
    volatile uint32_t ICSCR;       /* 0x04 */
    volatile uint32_t CFGR;        /* 0x08 */
    volatile uint32_t PLLCFGR;     /* 0x0C */
    /* PLLSAI1CFGR 0x10, PLLSAI2CFGR 0x14, CIER 0x18, CIFR 0x1C, CICR 0x20,
     * then one reserved word: six in total before AHB1RSTR at 0x28. */
    uint32_t RESERVED0[6];
    volatile uint32_t AHB1RSTR;    /* 0x28 */
    uint32_t RESERVED1[7];
    volatile uint32_t AHB1ENR;     /* 0x48 */
    volatile uint32_t AHB2ENR;     /* 0x4C */
    volatile uint32_t AHB3ENR;     /* 0x50 */
    uint32_t RESERVED2[1];
    volatile uint32_t APB1ENR1;    /* 0x58 */
    volatile uint32_t APB1ENR2;    /* 0x5C */
    volatile uint32_t APB2ENR;     /* 0x60 */
} RCC_TypeDef;

typedef struct {
    volatile uint32_t ACR;      /* 0x00 */
    volatile uint32_t PDKEYR;   /* 0x04 */
    volatile uint32_t KEYR;     /* 0x08 */
    volatile uint32_t OPTKEYR;  /* 0x0C */
    volatile uint32_t SR;       /* 0x10 */
    volatile uint32_t CR;       /* 0x14 */
    volatile uint32_t ECCR;     /* 0x18 */
    uint32_t RESERVED0[1];
    volatile uint32_t OPTR;     /* 0x20 */
} FLASH_TypeDef;

typedef struct {
    volatile uint32_t CR1;   /* 0x00 */
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t CR4;
    volatile uint32_t SR1;   /* 0x10 */
    volatile uint32_t SR2;   /* 0x14 */
} PWR_TypeDef;

typedef struct {
    volatile uint32_t CR1;   /* 0x00 */
    uint32_t RESERVED0[4];
    volatile uint32_t EGR;   /* 0x14 */
    uint32_t RESERVED1[3];
    volatile uint32_t CNT;   /* 0x24 */
    volatile uint32_t PSC;   /* 0x28 */
    volatile uint32_t ARR;   /* 0x2C */
} TIM_TypeDef;

#define RCC    ((RCC_TypeDef *)   RCC_BASE)
#define FLASH  ((FLASH_TypeDef *) FLASH_R_BASE)
#define PWR    ((PWR_TypeDef *)   PWR_BASE)
#define TIM2   ((TIM_TypeDef *)   TIM2_BASE)

/* RCC_CR */
#define RCC_CR_MSION      (1UL << 0)
#define RCC_CR_MSIRDY     (1UL << 1)
#define RCC_CR_HSION      (1UL << 8)
#define RCC_CR_HSIRDY     (1UL << 10)
#define RCC_CR_PLLON      (1UL << 24)
#define RCC_CR_PLLRDY     (1UL << 25)

/* RCC_CFGR */
#define RCC_CFGR_SW       (3UL << 0)
#define RCC_CFGR_SW_HSI   (1UL << 0)
#define RCC_CFGR_SW_PLL   (3UL << 0)
#define RCC_CFGR_SWS      (3UL << 2)
#define RCC_CFGR_SWS_HSI  (1UL << 2)
#define RCC_CFGR_SWS_PLL  (3UL << 2)
#define RCC_CFGR_HPRE     (0xFUL << 4)
#define RCC_CFGR_PPRE1    (7UL << 8)
#define RCC_CFGR_PPRE2    (7UL << 11)

/* RCC_PLLCFGR: M is bits 4..6 on L4 (three bits, M = field + 1) */
#define RCC_PLLCFGR_PLLSRC_HSI  (2UL << 0)
#define RCC_PLLCFGR_PLLM_Pos    4
#define RCC_PLLCFGR_PLLN_Pos    8
#define RCC_PLLCFGR_PLLR_Pos    25
#define RCC_PLLCFGR_PLLREN      (1UL << 24)

/* RCC_APB1ENR1 */
#define RCC_APB1ENR1_TIM2EN     (1UL << 0)
#define RCC_APB1ENR1_PWREN      (1UL << 28)

/* FLASH_ACR */
#define FLASH_ACR_LATENCY       (7UL << 0)
#define FLASH_ACR_LATENCY_0WS   (0UL << 0)
#define FLASH_ACR_LATENCY_4WS   (4UL << 0)
#define FLASH_ACR_PRFTEN        (1UL << 8)
#define FLASH_ACR_ICEN          (1UL << 9)
#define FLASH_ACR_DCEN          (1UL << 10)

/* FLASH_SR / FLASH_CR: 2 KB pages, 64-bit programming, single bank.
 * Confirmed against the device database that ships with STM32CubeProgrammer
 * (STM32_Prog_DB_0x435.xml): 128 sectors of 0x800, alignment 8. */
#define FLASH_SR_EOP        (1UL << 0)
#define FLASH_SR_OPERR      (1UL << 1)
#define FLASH_SR_PROGERR    (1UL << 3)
#define FLASH_SR_WRPERR     (1UL << 4)
#define FLASH_SR_PGAERR     (1UL << 5)
#define FLASH_SR_SIZERR     (1UL << 6)
#define FLASH_SR_PGSERR     (1UL << 7)
#define FLASH_SR_MISERR     (1UL << 8)
#define FLASH_SR_FASTERR    (1UL << 9)
#define FLASH_SR_OPTVERR    (1UL << 15)
#define FLASH_SR_BSY        (1UL << 16)
#define FLASH_SR_ERRORS     (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | \
                             FLASH_SR_PGAERR | FLASH_SR_SIZERR | FLASH_SR_PGSERR | \
                             FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_OPTVERR)

#define FLASH_CR_PG         (1UL << 0)
#define FLASH_CR_PER        (1UL << 1)
#define FLASH_CR_PNB_Pos    3
#define FLASH_CR_PNB        (0xFFUL << 3)
#define FLASH_CR_STRT       (1UL << 16)
#define FLASH_CR_LOCK       (1UL << 31)

#define FLASH_KEY1          0x45670123UL
#define FLASH_KEY2          0xCDEF89ABUL

/* Device electronic signature (readable by the CPU, not by the bootloader). */
#define L4_FLASH_SIZE_REG   (*(volatile uint16_t *)0x1FFF75E0UL)
#define L4_UID_REG          ((volatile uint32_t *)0x1FFF7590UL)

/* TIM */
#define TIM_CR1_CEN   (1UL << 0)
#define TIM_EGR_UG    (1UL << 0)

extern uint32_t SystemCoreClock;

#endif /* STM32L413_MIN_H */
