# Performance {#performance}

Every number on this page was measured on the board named, not modelled. The
raw CSV lives in `benchmark/results/` and `tools/report.py` regenerates these
tables from it.

## Go2, per call

18 DOF, 31 links, floating base. Microseconds, single precision, `-O3`.

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
| | | | | | |
| **torque tick** | 71 µs<br>14.1 kHz | 149 µs<br>6.7 kHz | 70 µs<br>14.2 kHz | 1033 µs<br>968 Hz | 1293 µs<br>773 Hz |
| **operational space** | 126 µs<br>8.0 kHz | 262 µs<br>3.8 kHz | 126 µs<br>7.9 kHz | 1564 µs<br>640 Hz | 1972 µs<br>507 Hz |
| **forward dynamics** | 184 µs<br>5.4 kHz | 382 µs<br>2.6 kHz | 145 µs<br>6.9 kHz | 2022 µs<br>495 Hz | 2567 µs<br>390 Hz |

Torque tick is `update_kinematics` + `rnea`. Operational space adds `crba`.
Forward dynamics is `update_kinematics` + the faster of ABA and CRBA for that
robot.

@note The G474, L413 and ESP32-C6 columns are current. The two RP2350 columns
were captured one release earlier, so their `rnea` and `gravity` read a few
percent slow — on an FPU core, and not on the Hazard3, where that change was
worth nothing. The Pico 2 image is the only one linked `copy_to_ram` and pays
no flash wait states, which is most of what separates a 150 MHz M33 from a
170 MHz M4F here.

## How it scales with the robot

STM32G474 at 170 MHz:

| Robot | nv | Torque tick | Operational space | Forward dynamics |
|---|---|---|---|---|
| `spine` | 9, floating | 26 µs — 38.0 kHz | 44 µs — 22.8 kHz | 59 µs — 16.9 kHz |
| `xarm7` | 7, fixed | 46 µs — 21.9 kHz | 84 µs — 11.8 kHz | 95 µs — 10.5 kHz |
| `go2` | 18, floating | 71 µs — 14.1 kHz | 126 µs — 8.0 kHz | 184 µs — 5.4 kHz |
| `g1` | 35, floating | 157 µs — 6.4 kHz | 353 µs — 2.8 kHz | 437 µs — 2.3 kHz |

`g1` is a Unitree G1: a 29-DOF humanoid, 40 links, floating base, and the
largest model in the suite. It closes a 6 kHz torque loop on a single
Cortex-M4F, and 3 kHz on the 80 MHz L413 inside 64 KB of RAM.

## Which forward-dynamics method

`update_kinematics` + the method, on two Cortex-M4F parts that agree to a
percentage point — so this is a property of the robot rather than of the board.

| Robot | nv | ABA | CRBA | winner |
|---|---|---|---|---|
| `xarm7` | 7, fixed | 257 µs | **198 µs** | CRBA, −23% |
| `spine` | 9, floating | **123 µs** | 128 µs | ABA, −4% |
| `go2` | 18, floating | **382 µs** | 435 µs | ABA, −12% |
| `g1` | 35, floating | **908 µs** | 1704 µs | ABA, −47% |

(µs on the L413 at 80 MHz.)

Two things move the line and they pull opposite ways. The Cholesky solve grows
as nv³ with nothing to skip, and a floating base makes that worse, because its
six DOF are ancestors of every joint and leave the mass matrix with no sparsity
to exploit. Against that, ABA's articulated-inertia congruence has a fast path
that needs a joint's origin to carry no rotation. Go2 takes it at all twelve
joints and the spine at all three. xarm7's origins are quarter turns, so it
takes the path at none of them and ABA stays on the general kernel — which is
why xarm7, at nv=7, is the one model where CRBA wins.

4% at spine's nv=9 is inside the range a different model would move, so measure
your own. The benchmark's `fd_crba` row against `aba` is exactly this
comparison.

## Pick a core with a hardware FPU

Measured on a single STM32L413 at 80 MHz by compiling the benchmark for
Cortex-M3 soft-float and running it on the same silicon: same clock, same
flash, same source, only `-mcpu` and `-mfloat-abi` differed.

| | slower without an FPU |
|---|---|
| `jacobian_world` | 6–10× |
| `update_kinematics` | 7–12× |
| `crba` | 7–16× |
| `aba` | 12–13× |
| `rnea` | **10–19×** |

The factor tracks arithmetic density, which is why `rnea` is worst. Flash grows
about 16 KB for the soft-float routines and the bigger code; RAM does not move,
because the data layout has nothing to do with the FPU.

The advice is a part with a single-precision FPU rather than a faster scalar
core: an 80 MHz M4F beats a 72 MHz M3 by more than ten times. For STM32 the
F303 and G431 are near price-and-pin equivalents of the F103.

## Footprint, and the smallest part it fits

With `--gc-sections`, float32, for `update_kinematics + rnea + jacobian + crba
+ gravity`, 16 links and 12 joints:

| | flash | RAM |
|---|---|---|
| M4F hard-float | 23.5 KB | 6.8 KB |
| Cortex-M3 soft-float | 39.6 KB | 6.8 KB |
| M3, `RD_ENABLE_ABA=0` | 39.6 KB | 5.2 KB |
| M0+ soft-float | 41.7 KB | 6.8 KB |

The whole library is 40.2 KB of Cortex-M4 code; the 23.5 KB above is what is
left once the linker drops what that program does not call.

Cortex-M0+ compiles and links, so there is no ARMv7-M or DSP dependency, and
`RD_USE_CMSIS_DSP` is off by default. **It fits an STM32F103C8** — 64 KB flash,
20 KB RAM.

Scaled to an F103 at 72 MHz, the torque tick comes out at 2.5 kHz for a 2-DOF
arm, 1.3 kHz for the spine, 650 Hz for xarm7, 380 Hz for Go2 and 160 Hz for G1.
Treat those as optimistic: the F103 has a prefetch buffer and no instruction
cache, and this library is fetch-bound. Gravity compensation and slow
trajectories, yes. Torque control on legs, no.

## Against code generation

Pinocchio can trace RNEA, ABA and CRBA symbolically and let CasADi emit
straight-line C for one specific robot: no loops, no traversal, every
subexpression eliminated. That is the strongest opponent available, so
`benchmark/codegen/` measures against it. Both sides are float32, `-O3`, same
compiler, same board, and both are checked against Pinocchio's own
double-precision answer.

Cycles per call on an STM32G474 at 170 MHz, `update_kinematics` plus the
algorithm against one generated call:

| | | RobotDynamics | Code generation | |
|---|---|---|---|---|
| `spine` | `rnea` | **4,843** | 12,498 | **2.6×** |
| 9 dof | `aba` | **10,089** | 27,211 | **2.7×** |
| | `crba` | **4,804** | 11,423 | **2.4×** |
| | `rnea` + `crba` | **7,805** | 23,927 | **3.1×** |
| `xarm7` | `rnea` | **8,312** | 25,354 | **3.1×** |
| 7 dof | `aba` | **21,094** | 30,671 | **1.5×** |
| | `crba` | **9,814** | 24,388 | **2.5×** |
| | `rnea` + `crba` | **15,011** | 49,743 | **3.3×** |
| `go2` | `rnea` | **13,058** | 58,120 | **4.5×** |
| 18 dof | `aba` | **31,526** | 79,780 | **2.5×** |
| | `crba` | **14,104** | 52,920 | **3.8×** |
| | `rnea` + `crba` | **22,419** | 111,042 | **5.0×** |

The lead widens with the robot, because the generated code grows with the model
while the library does not.

Size, `.text` of the compiled objects, same compiler and flags:

| | Cortex-M4 code |
|---|---|
| Generated `rnea` + `aba` + `crba`, `spine` only | 17,425 bytes |
| … `xarm7` only | 29,021 bytes |
| … `go2` only | 67,221 bytes |
| … all three | 113,667 bytes |
| **RobotDynamics, whole library, any robot** | **40,236 bytes** |

An in-order core with 32 FP registers turns the generated code's thousands of
simultaneously-live temporaries into stack traffic, and the size makes every
iteration an instruction fetch. Code generation trades size for arithmetic, and
that trade pays on a desktop: on an x86-64 host it wins by 1.1–1.7×.

The same head-to-head on an STM32L413 is in `benchmark/results/`. Its 128 KB of
flash fits Go2's generated code alone, where the G474's 512 KB fits all three
robots at once.

## Measuring your own

`benchmark/` has a port per board. On the STM32L413, for instance:

```bash
cd benchmark/stm32l4 && make run-bench PLL80=1
```

It writes a CSV that `tools/report.py` turns into the tables above. Bring your
own model through `tools/urdf2c.py` and it will be in the run.
