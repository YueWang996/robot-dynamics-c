# Performance {#performance}

All figures are measured. The raw CSV is in `benchmark/results/` and
`tools/report.py` generates these tables from it.

## Test conditions

| | |
|---|---|
| Precision | float32 |
| Optimisation | `-O3 -ffast-math` |
| Model | Go2 (Unitree quadruped), 18 DOF, 31 links, floating base |

## Per-function time

Microseconds.

| | **M4F @ 170**<br><sub>STM32G474</sub> | **M4F @ 80**<br><sub>STM32L413</sub> | **M33 @ 150**<br><sub>RP2350</sub> | **Hazard3 @ 150**<br><sub>RP2350</sub> | **RV32 @ 160**<br><sub>ESP32-C6</sub> |
|---|---|---|---|---|---|
| | FPU | FPU | FPU | *no FPU* | *no FPU* |
| `update_kinematics` | 26.9 | 55.5 | 24.9 | 265.3 | 354.7 |
| `fk_frame` | 10.7 | 22.2 | 9.9 | 107.0 | 142.1 |
| `jacobian_world` | 12.2 | 25.4 | 10.3 | 63.8 | 81.0 |
| `jacobian_local` | 19.9 | 41.4 | 16.7 | 177.6 | 224.2 |
| `rnea` | 44.1 | 93.3 | 45.4 | 767.9 | 938.4 |
| `crba` | 54.7 | 113.4 | 55.9 | 530.4 | 678.9 |
| `aba` | 157.5 | 326.9 | 120.2 | 1756.8 | 2211.9 |
| `gravity` | 33.1 | 67.9 | 32.7 | 314.3 | 471.5 |
| `spatial_acceleration` | 19.4 | 40.6 | 17.9 | 202.8 | 275.6 |
| `spatial_velocity` | 4.7 | 9.7 | 4.6 | 52.3 | 72.5 |

## Typical combinations

| Combination | Contains | G474 | L413 | RP2350 M33 | RP2350 Hazard3 | ESP32-C6 |
|---|---|---|---|---|---|---|
| Torque control | `update_kinematics` + `rnea` | 71 µs<br>14.1 kHz | 149 µs<br>6.7 kHz | 70 µs<br>14.2 kHz | 1033 µs<br>968 Hz | 1293 µs<br>773 Hz |
| Operational space | above + `crba` | 126 µs<br>8.0 kHz | 262 µs<br>3.8 kHz | 126 µs<br>7.9 kHz | 1564 µs<br>640 Hz | 1972 µs<br>507 Hz |
| Forward dynamics | `update_kinematics` + faster method | 184 µs<br>5.4 kHz | 382 µs<br>2.6 kHz | 145 µs<br>6.9 kHz | 2022 µs<br>495 Hz | 2567 µs<br>390 Hz |

@note The G474, L413 and ESP32-C6 columns are current. The two RP2350 columns
were captured one release earlier, so their `rnea` and `gravity` read a few
percent slow. The Pico 2 image is linked `copy_to_ram`, so it executes from RAM
with no flash wait states.

## Different model sizes

STM32G474 @ 170 MHz:

| Model | nv | Base | Torque control | Operational space | Forward dynamics |
|---|---|---|---|---|---|
| `spine` | 9 | floating | 26 µs / 38.0 kHz | 44 µs / 22.8 kHz | 59 µs / 16.9 kHz |
| `xarm7` | 7 | fixed | 46 µs / 21.9 kHz | 84 µs / 11.8 kHz | 95 µs / 10.5 kHz |
| `go2` | 18 | floating | 71 µs / 14.1 kHz | 126 µs / 8.0 kHz | 184 µs / 5.4 kHz |
| `g1` | 35 | floating | 157 µs / 6.4 kHz | 353 µs / 2.8 kHz | 437 µs / 2.3 kHz |

`g1` is a Unitree G1, a 29-DOF humanoid with 40 links. It reaches 6.4 kHz
torque control on a single Cortex-M4F, and 3 kHz on the 80 MHz L413 inside
64 KB of RAM.

## ABA against CRBA

`update_kinematics` plus the method, L413 @ 80 MHz:

| Model | nv | Base | ABA | CRBA | Gap |
|---|---|---|---|---|---|
| `xarm7` | 7 | fixed | 257 µs | **198 µs** | CRBA 23% faster |
| `spine` | 9 | floating | **123 µs** | 128 µs | ABA 4% faster |
| `go2` | 18 | floating | **382 µs** | 435 µs | ABA 12% faster |
| `g1` | 35 | floating | **908 µs** | 1704 µs | ABA 47% faster |

The same test on a G474 agrees to within one percentage point, so this is a
property of the model rather than the hardware.

Two factors pull in opposite directions:

- CRBA solves an `nv × nv` system, and the Cholesky factorisation grows as nv³.
  A floating base makes this worse: its six degrees of freedom are ancestors of
  every joint, which densifies the mass matrix and leaves no sparsity to
  exploit.
- ABA's articulated-inertia congruence has a fast path that requires a joint's
  origin to carry no rotation. All twelve of Go2's joints and all three of
  spine's qualify. xarm7's joint origins are quarter turns, so none do.

The 4% gap on `spine` can flip with a different model, so measure your own. The
benchmark's `fd_crba` row against `aba` is this comparison.

## With and without an FPU

Measured on one STM32L413 @ 80 MHz: the benchmark compiled for Cortex-M3
soft-float and run on the same silicon. Same clock, same flash, same source,
only `-mcpu` and `-mfloat-abi` differ.

| Function | Slower without an FPU |
|---|---|
| `jacobian_world` | 6–10× |
| `update_kinematics` | 7–12× |
| `crba` | 7–16× |
| `aba` | 12–13× |
| `rnea` | **10–19×** |

The factor tracks floating-point density. Flash grows about 16 KB; RAM does not
change.

Selection advice: prefer a core with a single-precision FPU over a faster
scalar core. An 80 MHz M4F beats a 72 MHz M3 by more than ten times. In the
STM32 range the F303 and G431 are close to the F103 in price and packaging.

## Code and memory footprint

With `--gc-sections`, float32, using only `update_kinematics`, `rnea`,
`jacobian`, `crba` and `gravity`, 16 links and 12 joints:

| Target | Flash | RAM |
|---|---|---|
| M4F hard-float | 23.5 KB | 6.8 KB |
| Cortex-M3 soft-float | 39.6 KB | 6.8 KB |
| Cortex-M3, `RD_ENABLE_ABA=0` | 39.6 KB | 5.2 KB |
| Cortex-M0+ soft-float | 41.7 KB | 6.8 KB |

The whole library is 40.2 KB of Cortex-M4 code; the table shows what remains
after the linker drops unused parts.

Cortex-M0+ compiles and links, so there is no ARMv7-M or DSP dependency.
**It fits an STM32F103C8** (64 KB flash, 20 KB RAM).

Scaled by cycle count to an F103 at 72 MHz, torque control rates are:

| Model | Rate |
|---|---|
| 2-DOF arm | 2.5 kHz |
| `spine` | 1.3 kHz |
| `xarm7` | 650 Hz |
| `go2` | 380 Hz |
| `g1` | 160 Hz |

These are optimistic. The F103 has a prefetch buffer and no instruction cache,
and this library is fetch-bound. Suitable for gravity compensation and slow
trajectory tracking, not for torque control on legs.

## Against code generation

Pinocchio can trace RNEA, ABA and CRBA symbolically and have CasADi emit
straight-line C for one robot: no loops, no tree traversal, all common
subexpressions eliminated. `benchmark/codegen/` measures against that. Both
sides are float32, `-O3`, same compiler, same board, and both are checked
against Pinocchio's own double-precision answer.

Cycles per call on an STM32G474 @ 170 MHz. The left column is
`update_kinematics` plus the algorithm; the right is one generated call:

| Model | Algorithm | RobotDynamics | Code generation | Ratio |
|---|---|---|---|---|
| `spine`<br>9 dof | `rnea` | **4,843** | 12,498 | **2.6×** |
| | `aba` | **10,089** | 27,211 | **2.7×** |
| | `crba` | **4,804** | 11,423 | **2.4×** |
| | `rnea` + `crba` | **7,805** | 23,927 | **3.1×** |
| `xarm7`<br>7 dof | `rnea` | **8,312** | 25,354 | **3.1×** |
| | `aba` | **21,094** | 30,671 | **1.5×** |
| | `crba` | **9,814** | 24,388 | **2.5×** |
| | `rnea` + `crba` | **15,011** | 49,743 | **3.3×** |
| `go2`<br>18 dof | `rnea` | **13,058** | 58,120 | **4.5×** |
| | `aba` | **31,526** | 79,780 | **2.5×** |
| | `crba` | **14,104** | 52,920 | **3.8×** |
| | `rnea` + `crba` | **22,419** | 111,042 | **5.0×** |

The lead grows with the model, because generated code grows with the model and
the library does not.

`.text` size of the compiled objects, same compiler and flags:

| | Cortex-M4 code |
|---|---|
| Generated `rnea` + `aba` + `crba`, `spine` only | 17,425 bytes |
| `xarm7` only | 29,021 bytes |
| `go2` only | 67,221 bytes |
| All three models | 113,667 bytes |
| **RobotDynamics, whole library, any model** | **40,236 bytes** |

Generated code is slower on an MCU for two reasons: it has thousands of
simultaneously live temporaries, which exceeds the Cortex-M4's 32 floating-point
registers and spills to the stack; and its size makes every iteration an
instruction fetch.

The same comparison on an x86-64 host goes the other way, where code generation
wins by 1.1–1.7×.

L413 figures are in `benchmark/results/`. Its 128 KB of flash fits the generated
code for Go2 alone; the G474's 512 KB fits all three models.

## Measuring your own model

`benchmark/` has one port per board. For the STM32L413:

```bash
cd benchmark/stm32l4 && make run-bench PLL80=1
```

It writes a CSV that `tools/report.py` turns into tables. A model converted
with `tools/urdf2c.py` is added to the run automatically.
