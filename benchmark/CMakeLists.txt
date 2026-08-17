cmake_minimum_required(VERSION 3.13)

# ============================================================================
# RobotDynamics benchmark for the Raspberry Pi Pico 2 (RP2350)
#
# The RP2350 ships two Cortex-M33 cores *and* two Hazard3 RISC-V cores on the
# same die, sharing the same clock tree, bus fabric and memories. Selecting
# between them changes only the CPU, which makes this board an unusually clean
# way to compare the two ISAs on identical silicon.
#
#   cmake -B build-arm   -DPICO_PLATFORM=rp2350-arm-s
#   cmake -B build-riscv -DPICO_PLATFORM=rp2350-riscv
# ============================================================================

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(rd_benchmark C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

set(RD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../RobotDynamics)

add_executable(rd_benchmark
    bench_main.c
    ${RD_DIR}/rd_algorithms.c
    ${RD_DIR}/rd_chain.c
    ${RD_DIR}/rd_state.c
)

target_include_directories(rd_benchmark PRIVATE
    ${RD_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(rd_benchmark PRIVATE
    BENCH_PICO=1
    RD_USE_SINGLE_PRECISION=1
    RD_USE_CMSIS_DSP=0
    RD_USE_STATIC_ALLOC=0
    RD_MAX_LINKS=40
    RD_MAX_JOINTS=24
)

# Identical optimisation settings on both architectures so the comparison
# isolates the CPU. -O3 rather than -Ofast: -ffast-math would change the
# floating-point semantics and muddy any later numerical validation.
target_compile_options(rd_benchmark PRIVATE
    -O3
    -Wall -Wextra
    -Wno-unused-parameter
    -Wno-missing-field-initializers
)

target_link_libraries(rd_benchmark PRIVATE
    pico_stdlib
    hardware_clocks
)

# Run out of SRAM. Executing from flash would make the measurement sensitive to
# XIP cache hit rate and to how each toolchain happens to lay out code, which
# would confound the Arm/RISC-V comparison.
pico_set_binary_type(rd_benchmark copy_to_ram)

pico_enable_stdio_usb(rd_benchmark 1)
pico_enable_stdio_uart(rd_benchmark 0)

pico_add_extra_outputs(rd_benchmark)
