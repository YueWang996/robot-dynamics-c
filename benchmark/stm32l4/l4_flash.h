/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file l4_flash.h
 * @brief Minimal STM32L4 flash programmer, used as the benchmark's only
 *        output channel.
 *
 * There is no SWD probe on this board, so semihosting is unavailable and the
 * results have to come back through the one path the ROM bootloader does
 * expose: main flash. The program writes its stdout into a reserved flash
 * region, resets, and the host reads that region back over USB DFU.
 *
 * The program/erase routines live in RAM (.RamFunc). On a single-bank L4 the
 * flash cannot be read while it is being written, so a routine that both drives
 * the operation and is fetched from the memory being written is asking for
 * trouble.
 */
#ifndef L4_FLASH_H
#define L4_FLASH_H

#include <stdint.h>

#define L4_FLASH_BASE       0x08000000UL
#define L4_FLASH_PAGE_SIZE  0x800UL          /* 2 KB, 128 pages, single bank */

/* Reserved for captured output: pages 96..111, i.e. 0x08030000 + 32 KB.
 * Well clear of the program, which is linked at 0x08010000. */
#define L4_OUT_ADDR         0x08030000UL
#define L4_OUT_PAGES        16UL
#define L4_OUT_SIZE         (L4_OUT_PAGES * L4_FLASH_PAGE_SIZE)

void     l4_flash_unlock(void);
void     l4_flash_lock(void);
uint32_t l4_flash_erase_pages(uint32_t first_page, uint32_t count);
uint32_t l4_flash_write_dw(uint32_t addr, uint32_t lo, uint32_t hi);

#endif /* L4_FLASH_H */
