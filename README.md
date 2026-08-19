# RobotDynamics

Rigid-body dynamics for microcontrollers. C99, no dependencies beyond `libm`,
single-precision by default, and fast enough to close a 1 kHz torque loop on an
18-DOF quadruped from a $5 board.

It is the embedded counterpart to
[bard](https://github.com/YueWang996/bard-pytorch-dynamics), which does the
same maths in PyTorch for training and simulation. Same model/data split, same
algorithm names, same URDFs — so a policy developed against bard on a
workstation can run against this library on the robot.

```c
#include "robot_dynamics.h"

rd_chain_t chain;
rd_chain_build(&my_model, &chain);

static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));

for (;;) {
    read_encoders(q_joints, qd);
    read_base_estimate(q_base);      /* NULL for a fixed-base robot */

    /* One traversal; everything below reads its cache. */
    rd_update_kinematics(&chain, &state, q_base, q_joints, qd);

    rd_rnea(&chain, &state, qdd, NULL, tau);              /* inverse dynamics */
    rd_jacobian(&chain, &state, eef, RD_FRAME_WORLD, J);

    write_torques(tau);
}
```

Nothing in that loop allocates. `qd`, `qdd` and `tau` are packed to length `nv`,
with the base in the first six elements for a floating base — the same layout
Pinocchio and bard use, so the vectors are interchangeable with them.

## Why it is shaped this way

A control tick usually needs several quantities at once — torques, a Jacobian,
maybe the mass matrix — and every one of them starts by walking the kinematic
tree. Computing that walk once and sharing it is most of the performance story.

So the library splits into three pieces. An `rd_model_t` is a static
description of the robot, generally generated from a URDF and living in flash.
`rd_chain_build` turns it once into an `rd_chain_t`, which pre-computes joint
offsets, spatial inertias, parent paths and a topological order. An
`rd_state_t` is the per-tick workspace: `rd_update_kinematics` fills it with
transforms, joint subspaces and spatial velocities, and every algorithm after
that reads from it.

The state buffer is caller-provided, so you decide where it lives, and it holds
every algorithm's scratch too — so nothing in the control loop allocates. On a
Go2, priming the cache costs 155 µs and each algorithm that follows costs
between 9 µs and 400 µs.

## Algorithms

| | RobotDynamics | bard |
|---|:---:|:---:|
| Shared kinematics cache | `rd_update_kinematics` | `update_kinematics` |
| Forward kinematics | `rd_forward_kinematics`, `rd_fk_frame` | `forward_kinematics` |
| Geometric Jacobian | `rd_jacobian` | `jacobian` |
| Inverse dynamics (RNEA) | `rd_rnea` | `rnea` |
| Forward dynamics (ABA) | `rd_aba` | `aba` |
| Mass matrix (CRBA) | `rd_crba` | `crba` |
| Spatial acceleration | `rd_spatial_acceleration` | `spatial_acceleration` |
| Spatial velocity | `rd_spatial_velocity` | — |
| Gravity / Coriolis terms | `rd_gravity`, `rd_nonlinear_terms`, `rd_coriolis` | — |
| Batching / autodiff / GPU | not applicable | yes |

Both world-frame and body-frame references are supported wherever a reference
frame makes sense. Revolute, prismatic, fixed and floating joints are
supported; joint axes must be axis-aligned.

**Every algorithm above is checked against Pinocchio** on the same URDFs, at
random configurations, in both precisions:

| Build | Worst relative error vs Pinocchio |
|---|---|
| `float64` | 5.5e-15 |
| `float32` | 1.9e-06 (float32 eps is 1.19e-07) |

Across a fixed-base arm, a floating-base serial chain and a floating-base
branched quadruped, comparing FK, spatial velocity, RNEA, ABA, gravity,
nonlinear terms, CRBA, both Jacobians and spatial acceleration. The error does
not grow with model size. Reproduce with `tools/validate.py`.

## Performance

> **Which build these are from.** The STM32L413 figures are current. The RP2350
> and STM32G474 figures were taken before the traversal caching described under
> [Where the time goes](#where-the-time-goes), so their `update_kinematics` --
> and therefore their control-loop budgets -- are pessimistic by up to ~40%.
> Neither board is currently connected, and re-measuring is the only honest way
> to update them. The saving is structural rather than a tuning trick, so it
> should carry across, but that is an expectation and not a measurement.

Microseconds per call, single precision, `-O3`, each part at its rated clock.
Columns: Raspberry Pi Pico 2 (RP2350) Cortex-M33 at 150 MHz, and an STM32G474
Cortex-M4F at 170 MHz. Four cores measured in total — see
[`benchmark/results/`](benchmark/results/).

| Algorithm | simple_arm<br><sub>2 dof</sub> | spine<br><sub>9 dof</sub> | xarm7<br><sub>7 dof</sub> | go2<br><sub>18 dof</sub> |
|---|---|---|---|---|
| `update_kinematics` | 16.18 | 23.17 | 55.48 | 155.17 |
| `fk_frame` | 9.71 | 13.08 | 29.18 | 14.81 |
| `jacobian_world` | 3.05 | 8.37 | 7.89 | 9.27 |
| `rnea` | 13.25 | 17.80 | 42.35 | 125.03 |
| `aba` | n/a | 59.03 | 136.77 | 399.64 |
| `crba` | 19.32 | 31.90 | 94.55 | 234.43 |
| `gravity` | 9.43 | 12.59 | 29.67 | 86.77 |
| `spatial_velocity` | 1.06 | 1.05 | 1.05 | 1.06 |

`simple_arm`'s URDF carries no inertial data, so `rd_aba` correctly refuses it
(`RD_ERR_SINGULAR`) and its dynamics columns are traversal cost only.

Go2 (18 DOF, 31 links) across the four cores measured so far:

| | M33 @ 150 MHz<br><sub>RP2350</sub> | M4F @ 170 MHz<br><sub>STM32G474</sub> | M4F @ 80 MHz<br><sub>STM32L413</sub> | Hazard3 @ 150 MHz<br><sub>RP2350</sub> |
|---|---|---|---|---|
| `update_kinematics` | 155.2 | 231.8 | 489.5 | 1458.8 |
| `rnea` | 125.0 | 129.7 | 270.7 | 1650.3 |
| `crba` | 234.4 | 459.5 | 971.5 | 2293.0 |
| `aba` | 399.6 | 617.7 | 1299.5 | 4239.1 |
| torque tick | 280 µs — 3.6 kHz | 361 µs — 2.8 kHz | 760 µs — 1.3 kHz | 3109 µs — 322 Hz |
| operational space | 515 µs — 1.9 kHz | 821 µs — 1.2 kHz | 1732 µs — 577 Hz | — |

The M4F needs 1.1–2.2× the M33's cycles for identical work even with flash
stalls removed, and the gap widens on the most data-heavy algorithms — this
library is memory-bound, so load/store throughput is what separates the two
cores. Flash wait states cost the G474 about 17% on top of that, concentrated
almost entirely in `update_kinematics` (~50%). That is *not* mainly the libm
calls, tempting as the correlation is: replacing `sinf`/`cosf` with a
polynomial removes only about 13% of the wait-state penalty on the L413. What
`update_kinematics` really has is the largest and most branch-diverse code
footprint per node, so it misses in the instruction cache more than the tight
inner loops of `rnea` do — `rnea` pays only 2%.

The two M4F parts agree to **within 4% on cycles per call** (median 0.98×) once
both are at four wait states, despite being separate boards with independently
written bring-up code — so the M4F figures are a property of the core, and any
other Cortex-M4F can be scaled from them by clock. Running the L413 at 16 MHz
with zero wait states reproduces the wait-state cost directly: a 5.00× clock
change buys 4.50× of wall clock, and the missing 0.50× is the flash.

A torque control tick — `update_kinematics` + `rnea` — and the loop rate it
allows:

| Robot | dof | Arm core | RISC-V core |
|---|---|---|---|
| `spine` | 9 | 41 µs — 24.4 kHz | 450 µs — 2.2 kHz |
| `xarm7` | 7 | 98 µs — 10.2 kHz | 1085 µs — 922 Hz |
| `go2` | 18 | 280 µs — 3.6 kHz | 3109 µs — 322 Hz |

Adding CRBA for operational-space control: Go2 515 µs (1.9 kHz). A
forward-dynamics tick (`update_kinematics` + `rd_aba`): Go2 555 µs (1.8 kHz).

**Run dynamics on the Arm cores.** The RP2350's RISC-V cores have no FPU, so
every float operation is emulated in software and the library runs 4–13x
slower there (median 9.8x). An integer-only control measurement in the same benchmark puts
RISC-V slightly *ahead* of Arm, which pins the difference entirely on floating
point rather than on the core.

Measured with `benchmark/`, which runs the same suite on a host and on both
RP2350 architectures; raw CSV is in
[`benchmark/results/`](benchmark/results/) and `tools/report.py` regenerates
these tables from it.

## Conventions

Get these backwards and nothing errors — you just get a wrong robot.

| | This library |
|---|---|
| Base quaternion | `q_base = [x y z qw qx qy qz]` — **scalar first**; Pinocchio and ROS are scalar-last |
| Base twist / acceleration | expressed in the root link's **body frame**, not world |
| Spatial vectors | `[linear, angular]` |
| `q_joints` | length `nj` |
| `qd`, `qdd`, `tau` | length **`nv`**, packed, base in the first six elements |
| `M` | `nv × nv`, fully filled (Pinocchio fills only the upper triangle) |
| Joint order | depth-first in URDF joint-declaration order — identical to Pinocchio and bard |

Because the joint ordering and the velocity-space layout match Pinocchio, `qd`,
`qdd` and `tau` vectors pass between the two libraries unchanged. The quaternion
does not.

`rd_gravity()` returns `g(q)`; `rd_nonlinear_terms()` returns `C(q,q̇)q̇ + g(q)`.
They are different functions.

## Building

```bash
cmake -B build
cmake --build build
./build/rd_test          # smoke test on the built-in spine model
```

To use it in your own project, add the directory as a subproject:

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

### Options

| Option | Default | Meaning |
|---|---|---|
| `RD_SINGLE_PRECISION` | `ON` | `float` rather than `double` for `rd_real_t` |
| `RD_CMSIS_DSP` | `OFF` | Use CMSIS-DSP for `sin`/`cos`/`sqrt`; needs `RD_CMSIS_DSP_INCLUDE_DIR`. `RD_FAST_TRIG` is both faster and far more accurate — see below |
| `RD_FAST_TRIG` | `OFF` | Polynomial `sin`/`cos`: 4.8x faster than libm at the same float32 accuracy |
| `RD_STATIC_ALLOC` | `OFF` | No `malloc` (see limitations) |
| `RD_OPTIMIZE_SIZE` | `OFF` | `-Os` rather than `-O3 -ffast-math` |
| `RD_ENABLE_DEBUG` | `OFF` | Assertion and log output |

`RD_MAX_LINKS` (16) and `RD_MAX_JOINTS` (12) bound the static model storage and
can be raised with a compile definition. Go2 needs at least 31 links.

### Where the time goes

`update_kinematics` dominates a control tick, and most of what it was computing
never changed. The motion subspace `S` is a function of the joint axis and the
link offset, so it is fixed the moment the chain is built — for every node. And
for a node whose joint cannot move, so are `T_parent_to_child` and its inverse:
two 4x4 SE3 composes and a matrix inverse per tick, arriving at the same numbers
forever.

That is not a corner case. A robot's fixture links — feet, sensor mounts,
inertial frames — are all fixed joints, and **Go2 is 18 fixed nodes out of 30**.
Computing these once per (chain, state) pairing, measured on the L413 at 80 MHz:

| | `update_kinematics` | torque tick | |
|---|---|---|---|
| spine (0 of 3 nodes fixed) | −12.4% | 118 → 109 µs | 8.5 → 9.2 kHz |
| xarm7 (3 of 10) | −22.6% | 273 → 233 µs | 3.7 → 4.3 kHz |
| go2 (18 of 30) | **−39.5%** | **760 → 567 µs** | **1.3 → 1.8 kHz** |

`rnea`, `crba`, `aba`, `fk_frame` and the Jacobians do not move at all — they
read the cache rather than build it. Spine has no fixed joints and still gains
12%, which is the `S` caching alone. With `RD_FAST_TRIG=1` on top, Go2's tick
reaches 545 µs (1.84 kHz), 28% below where it started.

The cache is keyed on the chain pointer, so driving two models through one state
buffer recomputes rather than quietly reading the other robot's transforms, and
`rd_test` checks that a warm state and a cold one produce bit-identical
kinematics at two different configurations. That check matters more than most:
if anything cached did turn out to depend on `q`, only the *second* call onwards
would be wrong, which no single-shot test would catch.

Two things that did **not** work, for the record. Shrinking the code to fit the
L4's 1 KB instruction cache — `rd_update_kinematics` compiles to 6.5 KB at
`-O3`, so every node re-fetches it — makes things much worse, because `-O3`'s
inlining is worth more than the fetch stalls it causes: `-O2` costs 38% and
`-Os` 54%. And running from RAM is slower than flash on Cortex-M4, because SRAM
at `0x20000000` goes over the system bus and gives up the Harvard split.

### Trigonometry

`update_kinematics` needs a sine and a cosine per revolute joint, and on a
Cortex-M4F libm is not cheap. Measured on an STM32L413 at 80 MHz — cycles for
one (sin, cos) pair, and worst-case absolute error against double-precision
libm over `[-pi, pi]`:

| | cycles | max abs error | |
|---|---|---|---|
| `sinf` + `cosf` | 273.6 | 5.9e-08 | the default |
| `sincosf` | 313.4 | 5.9e-08 | **slower** — newlib's is `bl sinf; bl cosf`, not fused |
| `arm_sin_f32` / `arm_cos_f32` | 105.5 | 1.9e-05 | `RD_CMSIS_DSP`: 512-entry table, 2 KB flash |
| `RD_FAST_TRIG=1` | 57.3 | 6.6e-08 | Cody-Waite reduction + Taylor series |

CMSIS-DSP interpolates linearly between table entries, which costs four decimal
digits of accuracy that float32 could otherwise resolve — for two thirds the
speed-up the polynomial gives. There is no case for using it here.

`RD_FAST_TRIG` is off by default because it changes results, however slightly,
and the shipped numbers are the validated ones. It passes the same Pinocchio
comparison at the same tolerances:

```bash
python3 tools/validate.py --urdf-root /path/to/bard --cflag=-DRD_FAST_TRIG=1
```

End to end on the L413, turning it on costs nothing and buys:

| | `update_kinematics` | `fk_frame` | `rnea` | torque tick |
|---|---|---|---|---|
| xarm7 | −9.6% | −15.8% | ±0% | 273 → 256 µs (−6.2%) |
| go2 | −8.2% | −10.3% | ±0% | 760 → 720 µs (−5.3%) |

`rnea`, `crba` and `aba` do not move at all — they read the cache
`update_kinematics` built and never call a transcendental. This is the shape of
the whole result: trig is worth roughly 9% of one routine, not a rewrite of the
library.

## Bringing in a robot

`tools/urdf2c.py` converts a URDF into a model header:

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h --floating-base
```

It enforces the constraints the C model has — axis-aligned joint axes,
15-character link names, parents declared before children — and fails loudly
rather than silently producing a wrong model. Generated headers for the four
benchmark platforms are in [`benchmark/models/`](benchmark/models/).

## Status and limitations

This is pre-1.0 and there are known gaps. Being explicit about them:

- **`RD_STATIC_ALLOC=ON` is still unsupported**, though for a narrower reason
  than before: the algorithms no longer allocate, but `rd_chain_build()` does,
  once, at startup. The control loop itself is allocation-free either way.
- **Prismatic joints are unvalidated.** The code path exists, but all four
  benchmark robots are revolute-only, so no numerical reference covers it.
- **Joint limits, damping and friction are parsed and stored but never used**
  by any algorithm.

## Testing

```bash
./build/rd_test                                          # smoke test, no deps
python3 tools/test_urdf2c.py                             # converter unit tests
python3 tools/test_urdf2c.py --urdf-root /path/to/bard   # + Pinocchio cross-checks
python3 tools/validate.py  --urdf-root /path/to/bard --double
```

The Pinocchio-backed tests skip themselves cleanly when `pinocchio` is not
installed, so the first two work anywhere.

`validate.py` covers FK, spatial velocity, RNEA, ABA, gravity, nonlinear terms,
CRBA, both Jacobians and spatial acceleration. Self-consistency checks alone are
not sufficient for changes to the maths — a wrong mass matrix can still be
symmetric and positive definite.

## Licence

[Apache License 2.0](LICENSE). Free for any use, commercial included: you can
ship it in a product, modify it, and keep your own changes closed. The only
obligations are keeping the licence and [NOTICE](NOTICE) with redistributions
and stating what you changed.

Apache 2.0 also carries an explicit patent grant from every contributor, which
is the main practical reason to prefer it over MIT for a library that anyone
may put into a commercial robot.

The sibling PyTorch library
[bard](https://github.com/YueWang996/bard-pytorch-dynamics) is MIT; the two are
compatible in either direction.

## Repository layout

```
RobotDynamics/       the library
  robot_dynamics.h   single-header entry point
  rd_model.h         robot description types
  rd_chain.[ch]      pre-processed kinematic tree
  rd_state.[ch]      per-tick cache + update_kinematics
  rd_algorithms.[ch] FK, Jacobian, RNEA, CRBA, spatial quantities
  rd_math.h          spatial algebra, inlined
  rd_config.h        precision, platform detection, size bounds
  spine_model.h      built-in example model
benchmark/           performance suite (host + RP2350) and captured results
.claude/skills/      agent skill: conventions, API, how to verify a change
LICENSE / NOTICE     Apache 2.0
tools/
  urdf2c.py          URDF -> rd_model_t header
  test_urdf2c.py     converter test suite
  validate.py        Pinocchio cross-check driver
  validate_dump.c    dumps library results for that comparison
  capture.py         flash an RP2350 and scrape its CSV report
  report.py          CSV -> the tables in this README
test_main.c          smoke test
```
