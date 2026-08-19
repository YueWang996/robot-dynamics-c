/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file l4_out.c
 * @brief stdout over flash, plus the two newlib syscalls this program needs.
 *
 * printf lands in _write, which stages bytes into an eight-byte buffer and
 * commits each full doubleword to the reserved flash region. Output is written
 * as it is produced rather than buffered to the end, so a run that faults
 * part-way still leaves everything printed up to that point readable.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "stm32l413.h"
#include "l4_flash.h"

static uint32_t s_cursor;          /* next flash address to program */
static uint8_t  s_stage[8];
static uint32_t s_staged;
static uint32_t s_error;           /* sticky: first flash error seen */
static int      s_open;

void l4_out_begin(void) {
    l4_flash_unlock();
    s_error  = l4_flash_erase_pages((L4_OUT_ADDR - L4_FLASH_BASE) / L4_FLASH_PAGE_SIZE,
                                    L4_OUT_PAGES);
    s_cursor = L4_OUT_ADDR;
    s_staged = 0;
    s_open   = 1;
    /* Unbuffered: no 1 KB stdio buffer on the heap, and every line reaches
     * flash before the next thing the program does can fault. */
    setvbuf(stdout, NULL, _IONBF, 0);
}

static void commit(void) {
    s_staged = 0;
    if (s_cursor + 8 > L4_OUT_ADDR + L4_OUT_SIZE) return;   /* region full */
    uint32_t lo, hi;
    memcpy(&lo, &s_stage[0], 4);
    memcpy(&hi, &s_stage[4], 4);
    uint32_t err = l4_flash_write_dw(s_cursor, lo, hi);
    if (err && !s_error) s_error = err;
    s_cursor += 8;
}

/* Pad the part-filled doubleword with 0xFF, which is what erased flash reads
 * as anyway. Flushing mid-stream therefore leaves 0xFF gaps, so the host drops
 * every 0xFF byte rather than just the trailing run -- the output is ASCII, so
 * none of them can be real. */
void l4_out_flush(void) {
    if (!s_open || s_staged == 0) return;
    while (s_staged < 8) s_stage[s_staged++] = 0xFF;
    commit();
}

void l4_out_end(void) {
    l4_out_flush();
    l4_flash_lock();
    s_open = 0;
}

uint32_t l4_out_error(void) { return s_error; }
uint32_t l4_out_used(void)  { return s_cursor - L4_OUT_ADDR; }

int _write(int fd, const char *buf, int len) {
    (void)fd;
    if (!s_open) return len;
    for (int i = 0; i < len; ++i) {
        s_stage[s_staged++] = (uint8_t)buf[i];
        if (s_staged == 8) commit();
    }
    return len;
}

/* newlib's malloc needs a heap. rd_chain_build is the only allocator in the
 * program and it checks its results, so running out here surfaces as a clean
 * "chain_build_failed" line rather than a crash. */
extern char end[];               /* first free byte, from the linker script */
extern char _estack[];
#define L4_STACK_RESERVE 3072u

void *_sbrk(int incr) {
    static char *heap;
    if (heap == 0) heap = end;
    char *limit = (char *)((uintptr_t)_estack - L4_STACK_RESERVE);
    if (heap + incr > limit) { errno = ENOMEM; return (void *)-1; }
    char *prev = heap;
    heap += incr;
    return prev;
}

int   _close(int f)                      { (void)f; return -1; }
int   _fstat(int f, struct stat *st)     { (void)f; st->st_mode = S_IFCHR; return 0; }
int   _isatty(int f)                     { (void)f; return 1; }
int   _lseek(int f, int p, int w)        { (void)f; (void)p; (void)w; return 0; }
int   _read(int f, char *b, int l)       { (void)f; (void)b; (void)l; return 0; }
int   _getpid(void)                      { return 1; }
int   _kill(int p, int s)                { (void)p; (void)s; errno = EINVAL; return -1; }
void  _exit(int c)                       { (void)c; for (;;) { } }
