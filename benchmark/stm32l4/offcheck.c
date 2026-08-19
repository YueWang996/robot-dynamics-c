/* SPDX-License-Identifier: Apache-2.0 */
/* Compile-time check of the hand-written register offsets against the
 * reference manual. A wrong one is otherwise silent -- a mis-placed
 * APB1ENR1 cost one probe run by leaving TIM2 unclocked. */
#include "stm32l413.h"
#define CHECK(type, field, off) \
    _Static_assert(__builtin_offsetof(type, field) == (off), #type "." #field)

CHECK(RCC_TypeDef,   CR,        0x00); CHECK(RCC_TypeDef,   ICSCR,     0x04);
CHECK(RCC_TypeDef,   CFGR,      0x08); CHECK(RCC_TypeDef,   PLLCFGR,   0x0C);
CHECK(RCC_TypeDef,   AHB1RSTR,  0x28); CHECK(RCC_TypeDef,   AHB1ENR,   0x48);
CHECK(RCC_TypeDef,   AHB2ENR,   0x4C); CHECK(RCC_TypeDef,   AHB3ENR,   0x50);
CHECK(RCC_TypeDef,   APB1ENR1,  0x58); CHECK(RCC_TypeDef,   APB1ENR2,  0x5C);
CHECK(RCC_TypeDef,   APB2ENR,   0x60);

CHECK(FLASH_TypeDef, ACR,       0x00); CHECK(FLASH_TypeDef, PDKEYR,    0x04);
CHECK(FLASH_TypeDef, KEYR,      0x08); CHECK(FLASH_TypeDef, OPTKEYR,   0x0C);
CHECK(FLASH_TypeDef, SR,        0x10); CHECK(FLASH_TypeDef, CR,        0x14);
CHECK(FLASH_TypeDef, ECCR,      0x18); CHECK(FLASH_TypeDef, OPTR,      0x20);

CHECK(TIM_TypeDef,   CR1,       0x00); CHECK(TIM_TypeDef,   EGR,       0x14);
CHECK(TIM_TypeDef,   CNT,       0x24); CHECK(TIM_TypeDef,   PSC,       0x28);
CHECK(TIM_TypeDef,   ARR,       0x2C);

CHECK(PWR_TypeDef,   CR1,       0x00); CHECK(PWR_TypeDef,   SR1,       0x10);
