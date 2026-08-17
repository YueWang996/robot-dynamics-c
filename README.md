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

static rd_real_t buf[RD_MAX_LINKS * 66 + 16];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));

for (;;) {
    read_encoders(q, qd);
    read_base_estimate(q_base, qd_base);          /* floating base only */

    /* One traversal; everything below reads its cache. */
    rd_update_kinematics_fb(&chain, &state, q_base, qd_base, q, qd);

    rd_rnea_cached(&chain, &state, qdd_base, qdd, NULL, tau);
    rd_jacobian_cached(&chain, &state, eef, RD_FRAME_WORLD, J);

    write_torques(tau);
}
```

Fixed-base robots call `rd_update_kinematics(&chain, &state, q, qd)` and pass
`NULL` for the base acceleration.

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

The state buffer is caller-provided, so you decide where it lives and it costs
nothing at runtime. On a Go2, priming the cache costs 224 µs and each algorithm
that follows costs between 9 µs and 517 µs.

## Algorithms

| | RobotDynamics | bard |
|---|:---:|:---:|
| Forward kinematics | `rd_fk_frame` | `forward_kinematics` |
| Shared kinematics cache | `rd_update_kinematics_fb` | `update_kinematics` |
| Geometric Jacobian | `rd_jacobian_cached` | `jacobian` |
| Inverse dynamics (RNEA) | `rd_rnea_cached` | `rnea` |
| Mass matrix (CRBA) | `rd_crba_cached` | `crba` |
| Spatial acceleration | `rd_spatial_acceleration_cached` | `spatial_acceleration` |
| Spatial velocity | `rd_get_spatial_velocity_cached` | — |
| Forward dynamics (ABA) | not implemented | `aba` |
| Batching / autodiff / GPU | not applicable | yes |

Both world-frame and body-frame references are supported wherever a reference
frame makes sense. Revolute, prismatic, fixed and floating joints are
supported; joint axes must be axis-aligned.

**Every algorithm above is checked against Pinocchio** on the same URDFs, at
random configurations, in both precisions:

| Build | Worst relative error vs Pinocchio |
|---|---|
| `float64` | 3.4e-15 |
| `float32` | 6.0e-07 (about five ULP; float32 eps is 1.19e-07) |

Across a fixed-base arm, a floating-base serial chain and a floating-base
branched quadruped. The error does not grow with model size. Details, the
convention mapping between the two libraries, and the bugs this caught are in
[`docs/VALIDATION.md`](docs/VALIDATION.md).

## Performance

Raspberry Pi Pico 2 (RP2350) at 150 MHz, single precision, `-O3`.
Microseconds per call on the **Arm** Cortex-M33 cores:

| Algorithm | simple_arm<br><sub>2 dof</sub> | spine<br><sub>9 dof</sub> | xarm7<br><sub>7 dof</sub> | go2<br><sub>18 dof</sub> |
|---|---|---|---|---|
| `update_kinematics` | 21.89 | 31.27 | 75.72 | 224.42 |
| `fk_frame` | 15.72 | 21.57 | 49.75 | 24.55 |
| `jacobian_world` | 2.91 | 8.25 | 7.75 | 9.15 |
| `rnea` | 20.53 | 26.79 | 58.65 | 170.27 |
| `crba` | 38.82 | 64.00 | 175.51 | 517.19 |
| `spatial_accel` | 8.49 | 10.47 | 22.01 | 62.71 |
| `spatial_velocity` | 0.99 | 1.00 | 1.00 | 0.99 |

A torque control tick — `update_kinematics` + `rnea` — and the loop rate it
allows:

| Robot | dof | Arm core | RISC-V core |
|---|---|---|---|
| `spine` | 9 | 58 µs — 17.2 kHz | 589 µs — 1.7 kHz |
| `xarm7` | 7 | 134 µs — 7.4 kHz | 1410 µs — 709 Hz |
| `go2` | 18 | 395 µs — 2.5 kHz | 4061 µs — 246 Hz |

**Run dynamics on the Arm cores.** The RP2350's RISC-V cores have no FPU, so
every float operation is emulated in software and the library runs 4–14x
slower there (median 10.3x). An integer-only control measurement in the same benchmark puts
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

- **`rd_gravity_compensation()` returns `C(q,q̇)q̇ + g(q)`, not `g(q)`.** It is
  currently the same call as `rd_nonlinear_terms()`. Getting gravity alone
  requires re-running `rd_update_kinematics` with `qd = 0`.
- **`RD_STATIC_ALLOC=ON` breaks RNEA and CRBA.** Both allocate scratch on every
  call, so under that flag they return `RD_ERR_ALLOC_FAILED`. This is also why
  the control loop is not yet allocation-free — the fix for both is to move the
  scratch into `rd_state_t`.
- **No forward dynamics.** ABA is in bard but not here.
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
