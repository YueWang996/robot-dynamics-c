# STM32G474 benchmark

Bare metal, no HAL. The only ST code used is the CMSIS device header and the
startup file, both from a Cube firmware package you already have. Results come
out over ARM semihosting, so this needs no UART pins and works on any G474 with
a SWD probe.

```bash
make CUBE=~/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1
make flash

# run and capture (openocd services the semihosting writes)
openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg \
  -c init -c "reset halt" -c "arm semihosting enable" -c resume
```

Swap `cmsis-dap.cfg` for `stlink.cfg` if that is your probe.

Default build is 170 MHz from flash, 4 wait states, ART prefetch and caches on —
the fastest configuration this part has. Two build flags exist only to explain
the numbers, and neither is how you would ship:

| | |
|---|---|
| `make ZEROWS=1` | stays on HSI16 at 16 MHz, where flash needs zero wait states. Same core, same code, no flash stalls — the control that separates flash cost from core cost. |
| `make RAM=1` | links the whole image to SRAM and loads it with the debugger. **Slower**, see below. |

## Running from SRAM is slower here, and that is expected

On Cortex-M4 the I-Code bus reaches the Code region (`0x08000000` flash) while
SRAM at `0x20000000` is reached over the System bus. Executing from SRAM makes
instruction fetch contend with data access on one bus and gives up the Harvard
split. This library is memory-bound, so the penalty lands hard: up to 61% more
cycles, and the damage scales with each algorithm's data traffic —
`update_kinematics` +0.5–11%, `crba` +40–61%.

The RP2350 build is linked `copy_to_ram` and does not suffer this, which is why
the `RAM=1` option exists at all: comparing the two boards naively would
otherwise credit the RP2350 with an advantage it does not have for this reason.
Use CCM SRAM at `0x10000000` if you want fast RAM execution on a G4 — it is in
the Code region — but it is only 32 KB against this image's 73 KB of text.

## Porting to another STM32

`startup_glue.c` is the whole platform layer: clock the core, start a 32-bit
timer at 1 MHz, enable the FPU. `bench_platform.h` reads `TIM2->CNT`.
`bench_main.c` needs no changes, and `tools/report.py` keys off the `arch`
column, so a new part appears as its own table automatically.
