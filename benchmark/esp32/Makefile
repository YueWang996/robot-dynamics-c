# SPDX-License-Identifier: Apache-2.0
#
# RobotDynamics benchmark for the ESP32 family (measured on an ESP32-C6).
#
# Built through the Arduino ESP32 core rather than a bare ESP-IDF project: the
# core *is* ESP-IDF, with the toolchain, the prebuilt libraries and esptool
# already assembled, and it is what was installed. No IDE is involved -- the
# arduino-cli inside the IDE bundle drives everything.
#
#   make build       # compile
#   make run         # compile, flash, capture the CSV
#   make capture     # re-read without reflashing (press reset first)
#
# CHIP selects the target; the sketch and platform layer are chip-agnostic.

CLI   ?= /Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli
PORT  ?= /dev/cu.usbmodem101
CHIP  ?= esp32c6
FREQ  ?= 160

ROOT  := $(abspath ../..)
SKETCH := rd_bench

# CDCOnBoot=cdc: the C6's only USB is the Serial/JTAG peripheral, and without
# this Serial goes to UART0 pins that nothing is connected to.
# huge_app: the default 1.2 MB partition is ample, but the four robot models
# and full printf leave less headroom than is comfortable.
FQBN := esp32:esp32:$(CHIP):CDCOnBoot=cdc,CPUFreq=$(FREQ),PartitionScheme=huge_app

# These go in build_opt.h rather than through --build-property, because
# arduino-cli runs a separate library-detection preprocessor pass that does not
# receive compiler.c.extra_flags -- it would fail to resolve the library
# includes and abort before compiling anything. build_opt.h is applied to both
# passes. It is generated because it has to carry absolute paths.
#
# -O3 sits at the end of the recipe, after the core's -Os, so it wins. Every
# other port in this benchmark is built -O3; matching it is the point.
DEFS := -DBENCH_ESP32=1 -DBENCH_NO_MAIN=1 -DRD_USE_SINGLE_PRECISION=1 \
        -DRD_MAX_LINKS=40 -DRD_MAX_JOINTS=24 \
        -I$(ROOT)/RobotDynamics -I$(ROOT)/benchmark -O3

$(SKETCH)/build_opt.h:
	@printf '%s\n' "$(DEFS)" > $@

build: $(SKETCH)/build_opt.h
	"$(CLI)" compile --fqbn "$(FQBN)" --warnings none $(SKETCH)

upload: build
	"$(CLI)" upload -p $(PORT) --fqbn "$(FQBN)" $(SKETCH)

run: upload
	python3 capture.py $(PORT) results.csv

capture:
	python3 capture.py $(PORT) results.csv

clean:
	rm -rf $(SKETCH)/build $(SKETCH)/build_opt.h results.csv

.PHONY: build upload run capture clean
