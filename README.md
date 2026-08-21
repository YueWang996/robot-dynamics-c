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
every algorithm's scratch too — so nothing in the control loop allocates.

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

Microseconds per call, single precision, `-O3`, on an STM32G474 (Cortex-M4F)
at 170 MHz:

| Algorithm | simple_arm<br><sub>2 dof</sub> | spine<br><sub>9 dof</sub> | xarm7<br><sub>7 dof</sub> | go2<br><sub>18 dof</sub> |
|---|---|---|---|---|
| `update_kinematics` | 7.28 | 10.79 | 18.81 | 30.19 |
| `fk_frame` | 7.18 | 9.55 | 18.26 | 10.64 |
| `jacobian_world` | 6.29 | 10.65 | 15.05 | 12.28 |
| `jacobian_local` | 8.73 | 16.88 | 21.95 | 19.91 |
| `rnea` | 14.04 | 18.25 | 32.28 | 51.44 |
| `aba` | n/a | 56.67 | 103.82 | 179.97 |
| `crba` | 10.32 | 18.62 | 40.49 | 60.86 |
| `gravity` | 9.80 | 12.64 | 22.05 | 34.50 |
| `spatial_acceleration` | 8.38 | 10.35 | 18.68 | 22.82 |
| `spatial_velocity` | 3.08 | 3.75 | 7.00 | 4.68 |

`simple_arm`'s URDF carries no inertial data, so `rd_aba` correctly refuses it
(`RD_ERR_SINGULAR`) and its dynamics columns are traversal cost only.

What that buys per control tick on the same part:

| Robot | dof | Torque tick | Operational space | Forward dynamics |
|---|---|---|---|---|
| `spine` | 9 | 29 µs — 34.4 kHz | 48 µs — 21.0 kHz | 67 µs — 14.8 kHz |
| `xarm7` | 7 | 51 µs — 19.6 kHz | 92 µs — 10.9 kHz | 123 µs — 8.2 kHz |
| `go2` | 18 | 82 µs — 12.3 kHz | 142 µs — 7.0 kHz | 210 µs — 4.8 kHz |

Torque tick is `update_kinematics` + `rnea`; operational space adds `crba`;
forward dynamics is `update_kinematics` + `aba`.

Go2 (18 DOF, 31 links) across the five cores measured so far, µs per call:

| | M4F @ 170<br><sub>G474</sub> | M33 @ 150<br><sub>RP2350</sub> | M4F @ 80<br><sub>L413</sub> | Hazard3 @ 150<br><sub>RP2350</sub> | RV32 @ 160<br><sub>ESP32-C6</sub> |
|---|---|---|---|---|---|
| | **FPU** | **FPU** | **FPU** | *no FPU* | *no FPU* |
| `update_kinematics` | **30.2** | 155.2 | 489.5 | 1458.8 | 1495.0 |
| `rnea` | **51.4** | 125.0 | 270.7 | 1650.3 | 2324.4 |
| `crba` | **60.9** | 234.4 | 971.5 | 2293.0 | 3627.7 |
| `aba` | **180.0** | 399.6 | 1299.5 | 4239.1 | 6243.3 |
| torque tick | **82 µs<br>12.3 kHz** | 280 µs<br>3.6 kHz | 567 µs<br>1.8 kHz | 3109 µs<br>322 Hz | 3819 µs<br>262 Hz |
| operational space | **142 µs<br>7.0 kHz** | 515 µs<br>1.9 kHz | 1732 µs<br>577 Hz | — | 7447 µs<br>134 Hz |

> Only the STM32G474 column is current. The others predate this round of
> optimisation and are pessimistic — by 2–3x on the dynamics, and by more than
> that on CRBA and the Jacobians.

### Where the remaining time goes

At 170 MHz the G474's flash needs four wait states, and the ART accelerator's
instruction cache is 1 KB — smaller than `rd_aba`'s inner loop, so that loop is
re-fetched from flash every iteration. Running the same binary with the
library's code in the part's CCM SRAM instead, which is on the I-Code bus at
zero wait states (`make CCMBENCH=1` in `benchmark/stm32g4`), separates the core
from the fetch. Go2:

| | flash, 4 WS | CCM SRAM | |
|---|---|---|---|
| `update_kinematics` | 38.6 µs | 33.3 µs | −13.8% |
| `rnea` | 51.4 µs | 50.1 µs | −2.6% |
| `crba` | 61.0 µs | 60.4 µs | −0.9% |
| `aba` | 179.9 µs | 142.1 µs | −21.0% |
| forward dynamics | 218.5 µs | 175.3 µs | −19.8% |

(measured before `update_kinematics` lost its address spilling, so its share
here is the older, larger figure; ABA is unchanged by that work.)

RNEA's and CRBA's loop bodies already fit that cache; ABA's is four times too
big. **If your part has tightly-coupled memory, put the dynamics code in it** —
on this one that is worth more than anything left in the source. The tables
above are measured from flash, so they stay comparable with the boards that
have no such memory.

Against a 16 MHz build where the flash needs no wait states at all
(`make ZEROWS=1`), the CCM figures come within 2.5%: at zero wait states this
code issues about one instruction per cycle, so there is no pipeline stall left
for hand-written assembly to remove.

**Run dynamics on a core with a hardware FPU.** Without one the library is
10–20× slower, which is a part-selection decision rather than something to
optimise around: for a quadruped, a Hazard3 or C3/C6/H2 class core is not a
viable target.

Raw CSV is in [`benchmark/results/`](benchmark/results/), and
`tools/report.py` regenerates these tables from it.

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
| `RD_CMSIS_DSP` | `OFF` | Use CMSIS-DSP for `sqrt`; needs `RD_CMSIS_DSP_INCLUDE_DIR`. Not for trigonometry — its table lookup is slower than `RD_FAST_TRIG` and 285× less accurate |
| `RD_FAST_TRIG` | `ON` | Polynomial `sin`/`cos`: 169 cycles per pair against libm's 521, measured inside `update_kinematics` on an STM32G474, at the same float32 accuracy. Worth 45% of that function on Go2. Turn off for libm's |
| `RD_STATIC_ALLOC` | `OFF` | No `malloc` (see limitations) |
| `RD_OPTIMIZE_SIZE` | `OFF` | `-Os` rather than `-O3 -ffast-math` |
| `RD_ENABLE_DEBUG` | `OFF` | Assertion and log output |

`RD_MAX_LINKS` (16) and `RD_MAX_JOINTS` (12) bound the static model storage and
can be raised with a compile definition. Go2 needs at least 31 links.

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
  rd_algorithms.[ch] FK, Jacobian, RNEA, ABA, CRBA, spatial quantities
  rd_math.h          spatial algebra, inlined
  rd_config.h        precision, platform detection, size bounds
  spine_model.h      built-in example model
benchmark/           performance suite and captured results, one
                     subdirectory per target board
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
