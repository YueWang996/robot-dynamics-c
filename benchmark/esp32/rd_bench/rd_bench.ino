/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RobotDynamics benchmark for the ESP32 family.
 *
 * Built through the Arduino ESP32 core rather than as a bare ESP-IDF project,
 * because that core *is* ESP-IDF with the toolchain, the prebuilt libraries
 * and esptool already assembled -- and it is what was installed. src/ holds
 * one shim per library source so each stays its own translation unit, exactly
 * as on the other ports; a unity build would inline across files and the
 * numbers would stop being comparable.
 *
 * See ../README.md for building and capturing without the IDE.
 */

#include <Arduino.h>

#include "bench_glue.h"

void setup() {
    Serial.begin(115200);

    /* USB CDC does not exist until the host opens the port, and anything
     * written before that is dropped. Wait for it, then leave the capture
     * script room to start reading. */
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 5000) delay(10);
    delay(2000);

    bench_cpu_hz = (uint32_t)getCpuFrequencyMhz() * 1000000UL;
    bench_run();
    Serial.flush();
}

void loop() {
    delay(1000);
}
