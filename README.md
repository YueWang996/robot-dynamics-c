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
Go2, priming the cache costs 224 µs and each algorithm that follows costs
between 9 µs and 747 µs.

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
branched quadruped. The error does not grow with model size. Details, the
convention mapping between the two libraries, and the bugs this caught are in
[`docs/VALIDATION.md`](docs/VALIDATION.md).

## Performance

Raspberry Pi Pico 2 (RP2350) at 150 MHz, single precision, `-O3`.
Microseconds per call on the **Arm** Cortex-M33 cores:

| Algorithm | simple_arm<br><sub>2 dof</sub> | spine<br><sub>9 dof</sub> | xarm7<br><sub>7 dof</sub> | go2<br><sub>18 dof</sub> |
|---|---|---|---|---|
| `update_kinematics` | 21.98 | 30.91 | 76.11 | 224.14 |
| `fk_frame` | 15.60 | 21.45 | 49.47 | 24.37 |
| `jacobian_world` | 3.06 | 8.57 | 8.00 | 9.47 |
| `rnea` | 17.33 | 23.44 | 56.99 | 171.47 |
| `aba` | n/a | 95.83 | 241.62 | 746.57 |
| `crba` | 37.45 | 59.16 | 176.19 | 505.24 |
| `gravity` | 11.39 | 15.23 | 36.13 | 106.54 |
| `spatial_velocity` | 1.05 | 1.06 | 1.06 | 1.05 |

`simple_arm`'s URDF carries no inertial data, so `rd_aba` correctly refuses it
(`RD_ERR_SINGULAR`) and its dynamics columns are traversal cost only.

A torque control tick — `update_kinematics` + `rnea` — and the loop rate it
allows:

| Robot | dof | Arm core | RISC-V core |
|---|---|---|---|
| `spine` | 9 | 54 µs — 18.4 kHz | 575 µs — 1.7 kHz |
| `xarm7` | 7 | 133 µs — 7.5 kHz | 1399 µs — 715 Hz |
| `go2` | 18 | 396 µs — 2.5 kHz | 4063 µs — 246 Hz |

A forward-dynamics tick (`update_kinematics` + `rd_aba`) is the expensive one:
Go2 lands at 971 µs, just clearing 1 kHz on an Arm core.

**Run dynamics on the Arm cores.** The RP2350's RISC-V cores have no FPU, so
every float operation is emulated in software and the library runs 4–13x
slower there (median 9.5x). An integer-only control measurement in the same benchmark puts
RISC-V slightly *ahead* of Arm, which pins the difference entirely on floating
point rather than on the core.

Full methodology, per-architecture tables, a host reference and a ranked list
of optimisation opportunities are in [`docs/PROFILING.md`](docs/PROFILING.md).

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
| `RD_CMSIS_DSP` | `OFF` | Use CMSIS-DSP for `sin`/`cos`/`sqrt`; needs `RD_CMSIS_DSP_INCLUDE_DIR` |
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
benchmark/           performance suite (host + RP2350)
docs/PROFILING.md    measured performance and analysis
docs/VALIDATION.md   Pinocchio cross-check, conventions, coverage
LICENSE / NOTICE     Apache 2.0
tools/
  urdf2c.py          URDF -> rd_model_t header
  test_urdf2c.py     converter test suite
  validate.py        Pinocchio cross-check driver
  validate_dump.c    dumps library results for that comparison
  capture.py         flash an RP2350 and scrape its CSV report
  report.py          CSV -> the tables in docs/PROFILING.md
test_main.c          smoke test
```
