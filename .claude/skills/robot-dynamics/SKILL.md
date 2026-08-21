---
name: robot-dynamics
description: Use the RobotDynamics C library — rigid-body kinematics and dynamics (FK, Jacobian, RNEA, ABA, CRBA) for microcontrollers. Load this before writing or reviewing any code that calls rd_* functions, builds an rd_model_t, converts a URDF for it, or reasons about whether a control loop will hit its rate. Also covers the conventions that silently produce wrong numbers if you get them backwards.
---

# RobotDynamics

C99 rigid-body dynamics sized for microcontrollers. Validated against Pinocchio
at machine precision. Companion to the PyTorch library
[bard](https://github.com/YueWang996/bard-pytorch-dynamics): same algorithm
names, same URDFs, same `q` ordering, so vectors move between them unchanged.

## The shape of every program that uses this

Three objects. A **model** is a static description (generated from a URDF, lives
in flash). `rd_chain_build` turns it once into a **chain** with joint offsets,
spatial inertias and a topological order precomputed. A **state** is the
per-tick workspace: `rd_update_kinematics` fills it, and every algorithm reads
it. Nothing allocates — the state is one caller-provided buffer.

```c
#include "robot_dynamics.h"

rd_chain_t chain;
rd_chain_build(&my_model, &chain);            /* once, at startup */

static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));

for (;;) {
    /* ONE traversal. Everything below reads its cache. */
    rd_update_kinematics(&chain, &state, q_base, q_joints, qd);

    rd_rnea(&chain, &state, qdd, NULL, tau);               /* tau from qdd */
    rd_jacobian(&chain, &state, eef, RD_FRAME_WORLD, J);
}

rd_chain_free(&chain);
```

Calling an algorithm without `rd_update_kinematics` first reads a stale cache
and returns plausible-looking wrong numbers. It will not error.

## Conventions that silently produce wrong answers

Get any of these backwards and nothing complains — you just get a wrong robot.

| | This library | Note |
|---|---|---|
| Base quaternion | `q_base = [x y z qw qx qy qz]` — **scalar first** | Pinocchio and ROS are scalar-**last**. Feeding a Pinocchio `q` straight in gives a plausible but wrong pose. |
| Base twist / accel frame | root link's **body frame** | not world. Matches Pinocchio's free-flyer. To convert a world twist: `rd_spatial_transform_motion_inv(state.T_dyn + root*16, v_world, qd)`. |
| Spatial vector order | `[linear, angular]` | same as Pinocchio |
| `q_joints` | length `nj`, joints only | configuration is split because nq ≠ nv |
| `qd`, `qdd`, `tau` | length **`nv`**, packed, base in the first six | `nv = 6 + nj` floating, `nj` fixed |
| `M` | `nv × nv` row-major, fully filled | Pinocchio fills only the upper triangle |
| `J` | `6 × nv` row-major | |
| `gravity` argument | 3-vector in **world** coordinates, or `NULL` for `(0,0,-9.81)` | |
| Joint order | depth-first in URDF joint-declaration order | identical to Pinocchio and bard, so `q` vectors are interchangeable |

Fixed-joint links are kept as zero-DOF nodes here; Pinocchio folds them into the
parent body. Link counts therefore differ (Go2: 31 nodes here, 13 joints in
Pinocchio) while the physics matches exactly.

## API

All of these take `(chain, state, ...)` and never allocate.

```c
/* per tick — call first */
rd_update_kinematics(chain, state, q_base, q_joints, qd);
    /* q_base NULL for a fixed base; q_joints NULL = zero config; qd NULL = at rest */

/* kinematics */
rd_forward_kinematics(chain, state, frame_id, T_out);        /* T_out[16], cached O(1) */
rd_fk_frame(chain, q_base, q_joints, frame_id, T_out);       /* standalone, no state */
rd_jacobian(chain, state, frame_id, ref, J_out);             /* J_out[6*nv] */
rd_spatial_velocity(chain, state, frame_id, ref, v_out);     /* v_out[6] */
rd_spatial_acceleration(chain, state, qdd, frame_id, ref, a_out);

/* dynamics */
rd_rnea(chain, state, qdd, gravity, tau_out);   /* inverse: tau = M qdd + C qd + g */
rd_aba(chain, state, tau, gravity, qdd_out);    /* forward: qdd = M^-1 (tau - C qd - g) */
rd_crba(chain, state, M_out);                   /* mass matrix */

/* convenience — all three are RNEA variants */
rd_gravity(chain, state, gravity, tau_out);         /* g(q) alone */
rd_nonlinear_terms(chain, state, gravity, tau_out); /* C(q,qd) qd + g(q) */
rd_coriolis(chain, state, tau_out);                 /* C(q,qd) qd, gravity removed */
```

`ref` is `RD_FRAME_WORLD` or `RD_FRAME_LOCAL`. Every function returns
`rd_status_t`; `RD_OK` is 0.

**`rd_gravity` is not `rd_nonlinear_terms`.** The first suppresses the cached
velocities and gives `g(q)`; the second keeps them and gives `C(q,q̇)q̇ + g(q)`.
Reaching for the wrong one is a classic gravity-compensation bug.

**`rd_aba` on a floating base**: `tau[0..5]` is the external wrench on the base
in its own body frame — pass zeros for a free-flying robot — and `qdd_out[0..5]`
is the resulting base acceleration. `rd_rnea` and `rd_aba` are exact inverses,
which is the cheapest self-check available:

```c
rd_rnea(&chain, &state, qdd, NULL, tau);
rd_aba (&chain, &state, tau, NULL, qdd_back);   /* qdd_back == qdd */
```

`rd_aba` returns `RD_ERR_SINGULAR` if a moving link has no inertia — usually a
URDF with no `<inertial>` block, not a bug in the caller.

## Adding a robot

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h [--floating-base]
```

The converter enforces what the C model can represent and fails loudly rather
than emitting something subtly wrong:

- **joint axes must be axis-aligned** (±X, ±Y, ±Z) — `rd_axis_t` cannot hold anything else
- link names are truncated to 15 chars and de-duplicated (`rd_link_t.name[16]`)
- parents are emitted before children — `rd_chain_build` depends on this
- revolute, continuous, prismatic and fixed joints only

The generated header documents its own `q[i] → joint name` mapping in the file
comment. Read it rather than guessing the order.

**The URDF must carry `<inertial>` blocks** if you want dynamics. A
kinematics-only URDF gives zero mass everywhere, `rd_aba` correctly refuses it,
and RNEA/CRBA return zeros that look like valid output.

If the model exceeds the defaults, raise `RD_MAX_LINKS` (16) and
`RD_MAX_JOINTS` (12) with compile definitions.

## Verifying a change

Never claim a change to the maths is correct without running this. Self-
consistency checks are not enough — a wrong mass matrix can still be symmetric
and positive definite, which is exactly how one real bug survived.

```bash
cmake -B build && cmake --build build && ./build/rd_test   # no dependencies
python3 tools/test_urdf2c.py                               # converter units
python3 tools/test_urdf2c.py --urdf-root /path/to/bard     # + Pinocchio checks
python3 tools/validate.py  --urdf-root /path/to/bard --double   # exact
python3 tools/validate.py  --urdf-root /path/to/bard            # shipping float32
```

`validate.py` compares FK, spatial velocity, RNEA, ABA, gravity, nonlinear
terms, CRBA, both Jacobians and spatial acceleration against Pinocchio on three
robots at random configurations. Expected: **~1e-15 in float64, a few ULP in
float32**. Anything worse means the change is wrong, not that the tolerance
needs relaxing.

Pinocchio comes from `pip install pin`; the tests skip themselves cleanly
without it.

## Performance envelope

Raspberry Pi Pico 2, one Arm Cortex-M33 at 150 MHz, single precision, µs/call:

| | spine (9 dof) | xarm7 (7 dof) | go2 (18 dof, 31 links) |
|---|---|---|---|
| `update_kinematics` | 23.2 | 55.5 | 155.2 |
| `rnea` | 17.8 | 42.4 | 125.0 |
| `crba` | 31.9 | 94.6 | 234.4 |
| `aba` | 59.0 | 136.8 | 399.6 |
| `jacobian` | 8.4 | 7.9 | 9.3 |

Control-loop budgets (Arm core, Go2): torque `update + rnea` 280 µs (3.6 kHz),
operational space `+ crba` 515 µs (1.9 kHz), forward dynamics `update + aba`
555 µs (1.8 kHz).

On an **STM32G474 (Cortex-M4F) at 170 MHz** the same Go2 tick costs 140 µs
(7.2 kHz) for torque and 253 µs (4.0 kHz) for operational space. The two M4F parts
agree within 4% on cycles per call (median 0.98×), so **scale any other
Cortex-M4F from these by clock** and expect to be close. The M4F needs 1.1–2.2×
the M33's cycles for this workload, and running from flash costs another ~17%.

Rules of thumb: `update_kinematics` scales with total link count, `rd_fk_frame`
with path depth only (~3 µs per level — much cheaper than a full update if you
need one frame), `aba` at ~13 µs per link.

**Only the STM32G474 figures are current.** Everything else here predates
folding fixed links out of the dynamics and is pessimistic by up to ~50% on
`rnea`, `crba` and `aba`. Say so rather than quoting them as current.

**Where the time actually goes.** `update_kinematics` dominates a tick and
everything else reads the cache it builds, so `rnea`, `crba`, `aba`, `fk_frame`
and the Jacobians are unaffected by anything done to it. Two facts follow:

- **Fixed joints are most of a real robot**, and cost almost nothing now. Feet,
  sensor mounts and inertial frames are all fixed nodes — Go2 is 18 of 30.
  `rd_chain_build` folds each one's inertia into its nearest moving ancestor and
  the dynamics traverse only the moving nodes, worth −53% on `rnea`, −47% on
  `crba` and −51% on `aba`.
- **Inertias are stored packed.** CRBA's composite is a sum of rigid-body
  inertias, which is itself a rigid body, so it lives in ten numbers rather
  than a 6x6 (−54%). ABA's articulated inertia is not a rigid body but is
  symmetric, so 21 numbers rather than 36 (−33%). Both are exact, not
  approximations. `rd_update_kinematics` refreshes only moving links
  too; a fixed link's pose and velocity are resolved by the accessor that asks
  for them. **Read frames through `rd_forward_kinematics` / `rd_spatial_velocity`,
  never from `state->T_world` directly** — for a fixed link that slot is not
  maintained. If a user reports it recomputing every tick, check
  they are not re-initialising the state each loop: that throws the cache away.
- **There is no separate link offset.** The link frame *is* the joint's child
  frame, so the motion subspace is the joint twist itself and `rd_axis_t` can
  only hold ±X/±Y/±Z. Anything assuming a link-offset transform is out of date.
- **libm is worth ~12% of `update_kinematics`**, and `RD_FAST_TRIG` is on by
  default now.
  Flash wait states cost `update_kinematics` ~50% on Cortex-M4F against 2% for
  `rnea`, but that is code footprint missing in the instruction cache, not the
  libm calls — removing them recovers only ~13% of the penalty.

Two dead ends worth not re-proposing: `-O2`/`-Os` to shrink code for the
instruction cache costs 38%/54%, because the inlining is worth more than the
fetch stalls; and linking to RAM is slower than flash on Cortex-M4, where SRAM
goes over the system bus and loses the Harvard split.

**A hardware FPU is the one hardware feature that decides whether this library
is usable.** RP2350's RISC-V cores are 4–13× slower than its Arm cores for this
reason alone — same die, same clock, and an integer-only control in the same
suite puts RISC-V slightly *ahead*. An **ESP32-C6** (RV32IMAC, no FPU, 160 MHz)
is 16–20× slower than the M33: Go2's torque tick is 3.8 ms, 262 Hz. If someone
asks whether a C3/C6/H2 can run this at a useful rate, the answer is no for a
quadruped and marginal for a 7-DOF arm; point them at an S3, an Arm part, or a
part with an FPU. It is a part-selection decision, not something to optimise
around.

## Constraints

- **`RD_STATIC_ALLOC=ON` does not work.** The algorithms never allocate, but
  `rd_chain_build` does, once at startup.
- **Prismatic joints are unvalidated** — the path exists, no reference covers it.
- **Joint limits, damping and friction** are parsed into the model and never
  used by any algorithm.
- **No inverse kinematics.**
- Models must satisfy `parent_idx < child_idx`. `urdf2c.py` guarantees it; a
  hand-written model can violate it and will be silently mis-ordered.

## Build options

| Option | Default | |
|---|---|---|
| `RD_SINGLE_PRECISION` | ON | `float` vs `double` for `rd_real_t` |
| `RD_CMSIS_DSP` | OFF | needs `RD_CMSIS_DSP_INCLUDE_DIR`. Do not use it for trig — its table lookup is slower *and* 285x less accurate than `RD_FAST_TRIG` |
| `RD_FAST_TRIG` | **ON** | polynomial `sin`/`cos`, 4.8x faster than libm at the same float32 accuracy. Set to 0 for libm's; both pass the Pinocchio comparison |
| `RD_STATIC_ALLOC` | OFF | see constraints |
| `RD_OPTIMIZE_SIZE` | OFF | `-Os` vs `-O3 -ffast-math` |

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```
