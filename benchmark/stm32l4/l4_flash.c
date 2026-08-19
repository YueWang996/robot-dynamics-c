/* SPDX-License-Identifier: Apache-2.0 */
#include "stm32l413.h"
#include "l4_flash.h"

#define RAMFUNC __attribute__((section(".RamFunc"), noinline, long_call))

/* Every error bit in FLASH_SR is sticky, and a leftover PGSERR from a previous
 * operation blocks the next one. Clear the lot before starting anything. */
RAMFUNC static void flash_wait_idle(void) {
    while (FLASH->SR & FLASH_SR_BSY) { }
}

void l4_flash_unlock(void) {
    flash_wait_idle();
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
}

void l4_flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

RAMFUNC uint32_t l4_flash_erase_pages(uint32_t first_page, uint32_t count) {
    uint32_t err = 0;
    for (uint32_t p = first_page; p < first_page + count; ++p) {
        flash_wait_idle();
        FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
        FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB)
                  | FLASH_CR_PER | (p << FLASH_CR_PNB_Pos) | FLASH_CR_STRT;
        flash_wait_idle();
        err |= FLASH->SR & FLASH_SR_ERRORS;
        FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB);
    }
    /* The instruction and data caches can hold lines that the erase just
     * invalidated underneath them; the reference manual requires a flush. */
    uint32_t acr = FLASH->ACR;
    FLASH->ACR = acr & ~(FLASH_ACR_ICEN | FLASH_ACR_DCEN);
    FLASH->ACR = acr;
    return err;
}

/* L4 programs one 64-bit doubleword at a time; two 32-bit stores, low half
 * first, and the operation starts on the second. */
RAMFUNC uint32_t l4_flash_write_dw(uint32_t addr, uint32_t lo, uint32_t hi) {
    flash_wait_idle();
    FLASH->SR = FLASH_SR_ERRORS | FLASH_SR_EOP;
    FLASH->CR |= FLASH_CR_PG;

    *(volatile uint32_t *)(addr)     = lo;
    __asm volatile ("dsb 0xF" ::: "memory");
    *(volatile uint32_t *)(addr + 4) = hi;

    flash_wait_idle();
    uint32_t err = FLASH->SR & FLASH_SR_ERRORS;
    FLASH->SR = FLASH_SR_EOP;
    FLASH->CR &= ~FLASH_CR_PG;
    return err;
}
