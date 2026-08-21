# SPDX-License-Identifier: Apache-2.0
#
# RobotDynamics benchmark for STM32L413 (Cortex-M4F), over USB DFU only.
#
# There is no SWD probe on this board, so this port does not use semihosting or
# openocd. The ROM bootloader is the whole toolchain: STM32CubeProgrammer
# downloads the image over DFU, starts it with the Go command, and reads the
# results back out of flash afterwards.
#
# The image is linked at 0x08010000 and the first flash word is left erased, so
# the boot ROM's empty-check returns the part to DFU on every reset. The board
# cannot be stranded.
#
#   make run-probe            # bring-up probe: RAM size, clock, flash writes
#   make run-bench            # the benchmark at 16 MHz  (cycle-count baseline)
#   make run-bench PLL80=1    # the benchmark at 80 MHz  (wall-clock figures)
#   make read                 # re-read the last results without re-running
#
# No Cube L4 package is needed: the register definitions are in stm32l413.h and
# the vector table is in startup_glue.c. The one file borrowed from a Cube
# install is core_cm4.h, which is generic ARM CMSIS-Core -- any package will do.

CUBE   ?= $(HOME)/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1
TC     ?= /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi
CPCLI  ?= /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI
RD     := ../../RobotDynamics
BENCH  := ..

# 0x08000000..0x0800FFFF stays erased on purpose -- that is what keeps the part
# falling into the ROM bootloader at every reset.
FLASH_ORIGIN ?= 0x08010000
FLASH_LEN_KB ?= 128

CPU    := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CFLAGS := $(CPU) -O3 -std=gnu11 -Wall -Wextra -Wno-unused-parameter \
          -ffunction-sections -fdata-sections \
          -DBENCH_STM32L4=1 -DBENCH_NO_MAIN=1 \
          -DRD_USE_SINGLE_PRECISION=1 -DRD_MAX_LINKS=40 -DRD_MAX_JOINTS=24 \
          -I$(CUBE)/Drivers/CMSIS/Include -I$(RD) -I$(BENCH) -I.

ifeq ($(PLL80),1)
  CFLAGS += -DBENCH_L4_PLL80=1
endif

# e.g. make run-bench PLL80=1 EXTRA=-DRD_FAST_TRIG=1
CFLAGS += $(EXTRA)

LDBASE := $(CPU) -Wl,--gc-sections -Wl,--no-warn-rwx-segments -nostartfiles \
          --specs=nano.specs --specs=nosys.specs -lc -lm
# The suite prints floats; the codegen head-to-head deliberately does not, so
# that build leaves newlib's float formatting out and spends the flash on the
# generated code instead.
ifneq ($(CODEGEN),1)
  LDBASE += -u _printf_float
endif

COMMON := startup_glue.c l4_flash.c l4_out.c offcheck.c

# CODEGEN=1 swaps the suite for the head-to-head against Pinocchio's
# code-generated dynamics. 128 KB of flash is the budget here, so generate the
# subset that fits: `python3 ../codegen/gen.py --robots go2 ...`.
ifeq ($(CODEGEN),1)
  CG := $(BENCH)/codegen
  CFLAGS += -I$(CG)/generated
  BENCH_SRCS := main.c $(COMMON) $(CG)/bench_codegen.c \
                $(wildcard $(CG)/generated/*.c) \
                $(RD)/rd_algorithms.c $(RD)/rd_chain.c $(RD)/rd_state.c
else
BENCH_SRCS := main.c $(COMMON) $(BENCH)/bench_main.c \
              $(RD)/rd_algorithms.c $(RD)/rd_chain.c $(RD)/rd_state.c
endif
# Same buffer sizes as every other port, so the L413 is not a special case.
BENCH_DEFS :=
# 64 KB, not the 48 KB of SRAM1 that STM32CubeProgrammer's device database
# lists: probe_main.c confirmed on target that SRAM2 mirrors directly above
# SRAM1 and that 0x20010000 faults.
BENCH_RAM  := 64

# Measures what sin/cos costs and what the alternatives buy. CMSIS-DSP comes
# from the G4 Cube package; --gc-sections drops all of arm_common_tables.c
# except the one 512-entry table arm_sin_f32 needs.
DSP        := $(CUBE)/Drivers/CMSIS/DSP
TRIG_SRCS  := trig_main.c $(COMMON) \
              $(DSP)/Source/FastMathFunctions/arm_sin_f32.c \
              $(DSP)/Source/FastMathFunctions/arm_cos_f32.c \
              $(DSP)/Source/CommonTables/arm_common_tables.c
TRIG_CFLAGS := -I$(DSP)/Include -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D_GNU_SOURCE \
               -DARM_DSP_CONFIG_TABLES -DARM_FAST_ALLOW_TABLES -DARM_TABLE_SIN_F32
TRIG_RAM   := 48

PROBE_SRCS := probe_main.c $(COMMON)
# 32 leaves 0x20008000 upward free to write test patterns into
PROBE_RAM  := 32

all: rd_bench.elf

define LDSCRIPT_RULE
	@printf 'ENTRY(Reset_Handler)\nMEMORY {\n  FLASH (rx) : ORIGIN = $(FLASH_ORIGIN), LENGTH = $(FLASH_LEN_KB)K\n  RAM  (xrw) : ORIGIN = 0x20000000, LENGTH = $(1)K\n}\n' > $(2)
	@cat link.ld.in >> $(2)
endef

link_bench.ld: link.ld.in Makefile
	$(call LDSCRIPT_RULE,$(BENCH_RAM),$@)

link_probe.ld: link.ld.in Makefile
	$(call LDSCRIPT_RULE,$(PROBE_RAM),$@)

link_trig.ld: link.ld.in Makefile
	$(call LDSCRIPT_RULE,$(TRIG_RAM),$@)

rd_bench.elf: $(BENCH_SRCS) link_bench.ld
	$(TC)-gcc $(CFLAGS) $(BENCH_DEFS) $(BENCH_SRCS) $(LDBASE) -T link_bench.ld \
	    -Wl,-Map=rd_bench.map -o $@
	@$(TC)-size $@
	@python3 ramcheck.py $@ $(BENCH_RAM)

rd_probe.elf: $(PROBE_SRCS) link_probe.ld
	$(TC)-gcc $(CFLAGS) $(PROBE_SRCS) $(LDBASE) -T link_probe.ld \
	    -Wl,-Map=rd_probe.map -o $@
	@$(TC)-size $@
	@python3 ramcheck.py $@ $(PROBE_RAM)

rd_trig.elf: $(TRIG_SRCS) link_trig.ld
	$(TC)-gcc $(CFLAGS) $(TRIG_CFLAGS) $(TRIG_SRCS) $(LDBASE) -T link_trig.ld \
	    -Wl,-Map=rd_trig.map -o $@
	@$(TC)-size $@

%.bin: %.elf
	$(TC)-objcopy -O binary $< $@

run-probe: rd_probe.bin
	python3 dfu.py run $< $(FLASH_ORIGIN)

run-bench: rd_bench.bin
	python3 dfu.py run $< $(FLASH_ORIGIN)

run-trig: rd_trig.bin
	python3 dfu.py run $< $(FLASH_ORIGIN)

read:
	python3 dfu.py read

clean:
	rm -f *.elf *.bin *.map link_bench.ld link_probe.ld link_trig.ld out.bin

.PHONY: all run-probe run-bench run-trig read clean
