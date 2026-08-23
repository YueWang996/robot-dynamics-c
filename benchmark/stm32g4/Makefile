# SPDX-License-Identifier: Apache-2.0
#
# RobotDynamics benchmark for STM32G474 (Cortex-M4F @ 170 MHz).
#
# Bare metal, no HAL: the only ST code used is the CMSIS device header and the
# startup file. Results come out over ARM semihosting, so this needs no UART
# pins and works on any G474 board with a SWD probe.
#
#   make CUBE=~/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1
#   make flash
#
#   # run and capture; openocd services the semihosting writes
#   openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg \
#     -c init -c "reset halt" -c "arm semihosting enable" -c resume
#
# Swap cmsis-dap.cfg for stlink.cfg if that is your probe.
#
# Default is 170 MHz from flash, 4 wait states, ART prefetch and caches on --
# the fastest configuration this part has. The flags below characterise it or
# swap a part of it out; none is how you would ship:
#
#   make CORDIC=1    sin/cos on the CORDIC coprocessor rather than the
#                    polynomial, through the RD_MATH_BACKEND hook
#   make TRIG=1      check rd_sincos against libm before timing anything
#   make SINGLE=1    build against dist/robot_dynamics.h, the file people
#                    actually download, rather than the source tree. With
#                    CORDIC=1 this is the end-to-end check that a backend
#                    reaches the single-header build.
#   make ZEROWS=1    HSI16 at 16 MHz, where flash needs zero wait states
#   make CCMBENCH=1  the library's hot code in CCM SRAM, on the I-Code bus at
#                    zero wait states. 170 MHz otherwise unchanged.
#   make RAM=1       link the image to SRAM and load it with the debugger.
#                    Slower: on Cortex-M4 SRAM at 0x20000000 is reached over
#                    the System bus, so instruction fetch contends with data
#                    access instead of using the I-Code bus to flash.

CUBE   ?= $(HOME)/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1
TC     ?= /Applications/ArmGNUToolchain/15.2.rel1/arm-none-eabi/bin/arm-none-eabi
RD     := ../../RobotDynamics
BENCH  := ..

CMSIS  := $(CUBE)/Drivers/CMSIS
DEV    := $(CMSIS)/Device/ST/STM32G4xx

CPU    := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CFLAGS := $(CPU) -O3 -std=gnu11 -Wall -Wextra -Wno-unused-parameter \
          -ffunction-sections -fdata-sections \
          -DSTM32G474xx -DBENCH_STM32G4=1 -DBENCH_NO_MAIN=1 \
          -DRD_USE_SINGLE_PRECISION=1 -DRD_MAX_LINKS=40 -DRD_MAX_JOINTS=32 \
          -I$(CMSIS)/Include -I$(DEV)/Include -I$(RD) -I$(BENCH) -I.

# RAM=1 links the image to run out of SRAM, matching how the RP2350 build is
# measured (copy_to_ram). Default is the normal flash image.
ifeq ($(ZEROWS),1)
  CFLAGS += -DBENCH_G4_ZEROWS=1
endif

# SINGLE=1 builds against dist/robot_dynamics.h rather than the source tree.
# -I$(DIST) goes ahead of the rest so "robot_dynamics.h" resolves to the
# shipped file; -I$(RD) stays reachable for backends/ and spine_model.h.
# dist/ is generated -- run  python3 ../../tools/amalgamate.py  first.
ifeq ($(SINGLE),1)
  DIST   := ../../dist
  CFLAGS := -I$(DIST) $(CFLAGS)
endif

# CORDIC=1 routes sin/cos to the G474's CORDIC coprocessor through the
# RD_MATH_BACKEND hook. See RobotDynamics/backends/rd_cordic_stm32g4.h. The
# quoting is what a header name has to survive to reach the preprocessor.
ifeq ($(CORDIC),1)
  CFLAGS += -DBENCH_G4_CORDIC=1 '-DRD_MATH_BACKEND="backends/rd_cordic_stm32g4.h"'
endif

# TRIG=1 checks rd_sincos against libm in double before the suite runs, and
# with CORDIC=1 also walks the coprocessor's precision field.
ifeq ($(TRIG),1)
  CFLAGS += -DBENCH_TRIG_ACCURACY=1
endif

ifeq ($(RAM),1)
  LDSCRIPT := STM32G474_RAM.ld
  CFLAGS   += -DBENCH_G4_RAM=1
else
  LDSCRIPT := STM32G474RETX_FLASH.ld
endif

# Appended last so it can override anything above, e.g. make EXTRA=-ffast-math
CFLAGS += $(EXTRA)

# rdimon = semihosting newlib. nano.specs keeps printf small enough to fit.
LDFLAGS := $(CPU) -T$(LDSCRIPT) -Wl,--gc-sections -Wl,-Map=rd_bench.map \
           --specs=nano.specs --specs=rdimon.specs -lc -lrdimon -lm -u _printf_float

# CCMBENCH=1 runs the library's hot code out of the G474's 32 KB CCM SRAM,
# which is on the I-Code bus and takes no wait states. Worth 21% on rd_aba at
# 170 MHz -- see ccm.ld. A characterisation option, not how the library ships:
# most parts have no tightly-coupled memory, so results/ is captured from flash.
ifeq ($(CCMBENCH),1)
  CFLAGS += -DBENCH_G4_CCM=1
endif

# CODEGEN=1 swaps the suite for the head-to-head against Pinocchio's
# code-generated dynamics. See ../codegen/README.md; the generated sources have
# to be produced first, they are not in the repository.
ifeq ($(CODEGEN),1)
  CG    := $(BENCH)/codegen
  CFLAGS += -I$(CG)/generated
  SRCS  := main.c startup_glue.c \
           $(CG)/bench_codegen.c $(wildcard $(CG)/generated/*.c) \
           $(RD)/rd_algorithms.c $(RD)/rd_chain.c $(RD)/rd_state.c
else ifeq ($(SINGLE),1)
SRCS := main.c startup_glue.c $(BENCH)/bench_main.c single_impl.c
else
SRCS := main.c startup_glue.c \
        $(BENCH)/bench_main.c \
        $(RD)/rd_algorithms.c $(RD)/rd_chain.c $(RD)/rd_state.c
endif
ASRC := $(DEV)/Source/Templates/gcc/startup_stm32g474xx.s

all: rd_bench.elf

$(LDSCRIPT):
	cp $(CUBE)/Projects/STM32G474E-EVAL1/Templates/STM32CubeIDE/STM32G474RETX_FLASH.ld $@
ifeq ($(CCMBENCH),1)
	sed '/The program code and other data into "ROM"/r ccm.ld' $@ > $@.tmp && mv $@.tmp $@
	grep -q _sccmram $@ || (echo "ccm.ld not spliced -- linker script layout changed"; exit 1)
endif

rd_bench.elf: $(SRCS) $(ASRC) $(LDSCRIPT)
	$(TC)-gcc $(CFLAGS) $(SRCS) $(ASRC) $(LDFLAGS) -o $@
	$(TC)-size $@

flash: rd_bench.elf
	openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg \
	  -c "program $< verify reset" -c shutdown

clean:
	rm -f rd_bench.elf rd_bench.map $(LDSCRIPT)

.PHONY: all flash clean
