/* SPDX-License-Identifier: Apache-2.0 */
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#include "bench_glue.h"

uint32_t bench_cpu_hz = 0;

int bench_printf(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    size_t len = (n < (int)sizeof(buf)) ? (size_t)n : sizeof(buf) - 1;
    Serial.write(reinterpret_cast<const uint8_t*>(buf), len);
    return n;
}
