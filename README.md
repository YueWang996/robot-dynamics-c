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

Microseconds per call, single precision, `-O3`. Raspberry Pi Pico 2 (RP2350),
one Cortex-M33 at 150 MHz:

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

Go2 (18 DOF, 31 links) across the five cores measured so far, µs per call:

| | M33 @ 150<br><sub>RP2350</sub> | M4F @ 170<br><sub>G474</sub> | M4F @ 80<br><sub>L413</sub> | Hazard3 @ 150<br><sub>RP2350</sub> | RV32 @ 160<br><sub>ESP32-C6</sub> |
|---|---|---|---|---|---|
| | **FPU** | **FPU** | **FPU** | *no FPU* | *no FPU* |
| `update_kinematics` | 155.2 | 231.8 | 489.5 | 1458.8 | 1495.0 |
| `rnea` | 125.0 | 129.7 | 270.7 | 1650.3 | 2324.4 |
| `crba` | 234.4 | 459.5 | 971.5 | 2293.0 | 3627.7 |
| `aba` | 399.6 | 617.7 | 1299.5 | 4239.1 | 6243.3 |
| torque tick | 280 µs<br>3.6 kHz | 361 µs<br>2.8 kHz | 567 µs<br>1.8 kHz | 3109 µs<br>322 Hz | 3819 µs<br>262 Hz |
| operational space | 515 µs<br>1.9 kHz | 821 µs<br>1.2 kHz | 1732 µs<br>577 Hz | — | 7447 µs<br>134 Hz |

Torque tick is `update_kinematics` + `rnea`; operational space adds `crba`. The
RP2350 and STM32G474 columns were taken before the `update_kinematics`
optimisation and are pessimistic on that row by up to 40%.

A torque tick on the M33 and the RP2350's RISC-V core, across robots:

| Robot | dof | Arm core | RISC-V core |
|---|---|---|---|
| `spine` | 9 | 41 µs — 24.4 kHz | 450 µs — 2.2 kHz |
| `xarm7` | 7 | 98 µs — 10.2 kHz | 1085 µs — 922 Hz |
| `go2` | 18 | 280 µs — 3.6 kHz | 3109 µs — 322 Hz |

**Run dynamics on a core with a hardware FPU.** Without one the library is
10–20× slower, which is a part-selection decision rather than something to
optimise around: for a quadruped, a Hazard3 or C3/C6/H2 class core is not a
viable target. Two Cortex-M4F parts agree within 4% on cycles per call, so
another M4F can be scaled from these by clock.

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
| `RD_FAST_TRIG` | `OFF` | Polynomial `sin`/`cos`: 57 cycles per pair against libm's 274 on Cortex-M4F, same float32 accuracy. Worth 5–6% of a torque tick |
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
