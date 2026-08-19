/* SPDX-License-Identifier: Apache-2.0 */
/*
 * C linkage for the benchmark, kept out of the .ino on purpose: arduino-cli
 * injects prototypes for setup() and loop() after the last preprocessor line
 * of a sketch, and an `extern "C"` block there would swallow them and give
 * them the wrong linkage.
 */
#ifndef BENCH_GLUE_H
#define BENCH_GLUE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** CPU clock in hertz, read by bench_platform.h. Set before bench_run(). */
extern uint32_t bench_cpu_hz;

/** bench_platform.h redirects the suite's printf here; writes to Serial. */
int bench_printf(const char* fmt, ...);

/** The benchmark itself, in ../../bench_main.c. */
int bench_run(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_GLUE_H */
