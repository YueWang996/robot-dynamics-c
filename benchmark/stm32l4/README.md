# RobotDynamics benchmark — STM32L413RC

Cortex-M4F at 80 MHz, 256 KB flash, 64 KB RAM. Results:
[`../results/stm32l413.csv`](../results/stm32l413.csv).

This port has **no debugger**. The board's only connection is USB, and the chip
exposes nothing there but the ROM bootloader in DFU mode, so semihosting — how
the [G474 port](../stm32g4/) reports — is not available. The bootloader is
therefore the entire toolchain: it downloads the image, starts it, and reads the
results back out of the flash the program wrote them to.

```bash
make run-probe            # bring-up probe: RAM size, clock, flash writes
make run-bench PLL80=1    # the benchmark at 80 MHz
make run-bench            # the same image at 16 MHz, zero wait states
make read                 # re-read the last results without re-running
```

Everything is local: `arm-none-eabi-gcc` and the `STM32_Programmer_CLI` that
ships inside the STM32CubeProgrammer app bundle. No openocd, no dfu-util, no
Cube L4 package — the register definitions are in
[`stm32l413.h`](stm32l413.h) and the vector table is in
[`startup_glue.c`](startup_glue.c). The one borrowed file is `core_cm4.h`,
which is generic ARM CMSIS-Core; any Cube package has it.

## Why the image is linked at 0x08010000

The first flash word is left erased on purpose. The L4 boot ROM's empty-check
sends the part to the system bootloader whenever `0x08000000` reads as
`0xFFFFFFFF`, so **every reset comes back as a USB DFU device** no matter what
the program does. `STM32_Programmer_CLI -g 0x08010000` starts it, the program
resets itself when it is finished, and the host reads the results.

There is no way to strand the board, which matters when the bootloader is also
the only debug channel. A firmware that hangs or faults costs a power cycle.

```
0x08000000  erased, permanently        <- keeps the part falling into DFU
0x08010000  program (128 KB region, 73 KB used)
0x08030000  captured stdout (32 KB region, ~2.6 KB used)
```

`printf` lands in `_write` ([`l4_out.c`](l4_out.c)), which stages bytes into an
eight-byte buffer and commits each full doubleword to the result region. Output
is written as it is produced, not buffered to the end, so a run that faults
part-way still leaves everything printed up to that point readable — which is
how the fault marker in the probe below survives. The program/erase routines
live in RAM (`.RamFunc`): on a single-bank L4 the flash cannot be read while it
is being written.

## Run the probe first

[`probe_main.c`](probe_main.c) answers the questions that would otherwise be
silent failures, and proves the whole download / Go / write / read-back loop
while doing it:

```
RDL4PROBE1
clk_hz,16000000
flash_size_kb,256
spin_1e6_us,187500                          <- exactly 3 cycles/iteration at 16 MHz
ram,2,sram1_top,0x2000BFF8,ok
ram,4,sram2_mirror,0x2000C000,ok
ram,5,sram2_mirrortop,0x2000FFF8,ok
sram2_aliases_mirror,yes
ram,7,past_64k,0x20010000,  !FAULT step=7   <- expected: nothing above 64 KB
```

**RAM is 64 KB contiguous from `0x20000000`**, not the 48 KB that
STM32CubeProgrammer's device database lists for device 0x435 — that figure is
SRAM1 alone, and SRAM2 really does mirror directly above it. The benchmark needs
about 52 KB, so the difference decides whether the build fits at all. The probe
ends by faulting past 64 KB on purpose; the fault handler records the step and
resets, so the marker is a successful result, not a crash.

The probe earned its keep immediately: the first run reported `spin_1e6_us,0`,
which was a wrong `RESERVED0[9]` in `RCC_TypeDef` putting `APB1ENR1` at 0x64
instead of 0x58. TIM2's clock enable went nowhere and the timer never ran — a
mistake that would have produced confidently wrong timings. The offsets are now
asserted at compile time in [`offcheck.c`](offcheck.c).

## Confirming the numbers mean something

Two hand-written bring-ups on two different boards, running the same code, are
each other's check:

| | |
|---|---|
| L413 @ 80 MHz vs G474 @ 170 MHz, cycles/call | median **0.98x**, range 0.96–1.00x |
| Cost of 4 wait states (16 MHz/0WS → 80 MHz/4WS) | median 1.11x, up to 1.55x |
| Wall-clock speed-up 16 → 80 MHz | median 4.50x, against a 5.00x clock ratio |

The same image is built for both clocks, so the cycle counts have to agree
except for flash wait states — and they do, to within 4% of a completely
separate board. The shortfall from a 5.00x speed-up is exactly the wait-state
cost. `make run-bench` without `PLL80=1` reproduces the zero-wait-state side.

That cost falls almost entirely on `update_kinematics` and `fk_frame`, which
are also the only two routines calling libm — but the two facts turn out to be
mostly unrelated. `make run-trig` measures sin/cos directly, and building the
benchmark with `EXTRA=-DRD_FAST_TRIG=1` removes the libm calls entirely:

| go2 `update_kinematics` | 0 WS | 4 WS | penalty |
|---|---|---|---|
| libm | 25,844 | 39,163 | 1.52x |
| polynomial | 24,424 | 35,954 | 1.47x |

Dropping libm recovers only about 13% of the 13,319-cycle penalty. What
`update_kinematics` actually has is the largest and most branch-diverse code
footprint per node, so it misses in the instruction cache far more than
`rnea`'s tight inner loops, which pay 2%.

`make run-trig` also settles what to use for the trig itself — cycles for one
(sin, cos) pair, and worst-case error against double-precision libm:

| | 4 WS | 0 WS | max abs error |
|---|---|---|---|
| `sinf` + `cosf` | 273.6 | 246.4 | 5.9e-08 |
| `sincosf` | 313.4 | 280.1 | 5.9e-08 |
| CMSIS-DSP table | 105.5 | 94.4 | 1.9e-05 |
| polynomial | 57.3 | 57.3 | 6.6e-08 |

newlib's `sincosf` is slower than calling both, because it is literally
`bl sinf; bl cosf` plus stack shuffling. The polynomial is the only one that
costs the same at both wait-state settings: no table, no branches, small enough
to stay in cache.

## Re-measuring after a library change

`make run-bench PLL80=1` is the shipped configuration and what
[`../results/stm32l413.csv`](../results/stm32l413.csv) holds. Two variations are
worth knowing:

```bash
make run-bench                          # 16 MHz, zero wait states
make run-bench PLL80=1 EXTRA=-DRD_FAST_TRIG=1
```

`EXTRA` appends to `CFLAGS`, and a later `-O` wins over the default `-O3`, so
`EXTRA=-O2` also works for checking an optimisation-level question — the answer
turned out to be that `-O2` costs 38% and `-Os` 54%, because `-O3`'s inlining is
worth more than the instruction-cache misses it causes.

## Notes

- `ramcheck.py` fails the build if data + bss + heap + stack exceeds the part.
  The linker cannot catch this on its own: `rd_chain_build` allocates about
  11 KB for Go2 at run time and no section describes it.
- The clock is programmed explicitly rather than assumed. Arriving via Go rather
  than via reset means the bootloader has already configured the tree, typically
  HSI48 for USB, so reset defaults do not hold.
- `_sbrk` bounds the heap against the stack, so exhausting it surfaces as a
  clean `chain_build_failed` line instead of a crash on a board with no
  debugger.
