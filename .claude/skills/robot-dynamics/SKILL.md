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

## Contacts, geared joints, and the solve

Three things exist for building a controller on top, and all three cost nothing
when unused -- that was most of the work in adding them.

**External forces.** `rd_rnea_ext(chain, state, qdd, gravity, f_ext, tau)` and
`rd_aba_ext(chain, state, tau, gravity, f_ext, qdd)`. `f_ext` is `[6*n_nodes]`
or NULL, one spatial force per **node** -- the indexing `rd_forward_kinematics`
uses, not the velocity indexing of `tau` -- ordered `[linear, angular]` in that
link's **own body frame**. It is the force the world applies to the link.

Node indexing is what lets a contact sit where it physically does: feet and
fingertips are usually *fixed* links, which `rd_chain_build` folds into the
moving link above them, and a force placed on one is carried there
automatically. To convert a world-frame contact force, rotate it by the
transpose of the link's world rotation from `rd_forward_kinematics`.

RNEA is linear in `f_ext`, so it runs as its own inward pass and the NULL path
is byte-for-byte the old function. ABA cannot separate it and pays ~1%.

**Armature.** `rd_chain_set_armature(chain, vidx, n^2 * I_rotor)`. Reflected
rotor inertia, added to M's diagonal, to `tau` via `qdd`, and to the
articulated inertia, so CRBA, RNEA and ABA stay consistent. **Tell anyone with
a geared robot about this**: at a servo's reduction ratio the rotor term is
routinely larger than the link, and without it M(q) comes out systematically
small. URDF has no field for it, so it starts at zero.

**Cholesky.** `rd_cholesky_factor(A, n, dinv)` overwrites A's lower triangle
with L; `rd_cholesky_solve(L, dinv, b, x, n)` then solves for as many
right-hand sides as wanted. That split is the point -- operational-space
inertia is one factorisation and six solves.

There is **no controller and no contact solver here**, deliberately. A WBC's
shape is a choice about control architecture; this library supplies M, h, J,
`J_dot*qd` (that is `rd_spatial_acceleration` with `qdd = NULL`) and the
factorisation, and stops.

## Performance envelope

STM32G474, one Arm Cortex-M4F at 170 MHz, single precision, µs/call:

| | spine (9 dof) | xarm7 (7 dof) | go2 (18 dof, 31 links) | g1 (35 dof, 40 links) |
|---|---|---|---|---|
| `update_kinematics` | 10.4 | 17.8 | 26.9 | 59.9 |
| `rnea` | 15.9 | 27.7 | 44.1 | 97.3 |
| `crba` | 17.5 | 38.9 | 54.7 | 196.1 |
| `aba` | 48.8 | 105.8 | 157.5 | 377.0 |
| `fd_crba` | 51.1 | 77.1 | 180.5 | 750.0 |
| `jacobian_world` | 10.6 | 15.1 | 12.2 | 26.0 |

Control-loop budgets on that part:

| | spine | xarm7 | go2 | g1 |
|---|---|---|---|---|
| torque `update + rnea` | 26 µs / 38.0 kHz | 46 µs / 21.9 kHz | 71 µs / 14.1 kHz | 157 µs / 6.4 kHz |
| operational space `+ crba` | 44 µs / 22.8 kHz | 84 µs / 11.8 kHz | 126 µs / 8.0 kHz | 353 µs / 2.8 kHz |
| forward dynamics, best method | 59 µs / 16.9 kHz | 95 µs / 10.5 kHz | 184 µs / 5.4 kHz | 437 µs / 2.3 kHz |

`g1` is a Unitree G1 humanoid, 29 actuated joints on a floating base, and the
largest model in the suite. **A 29-DOF humanoid closes a 6 kHz torque loop on
one Cortex-M4F.** Its forward dynamics is the clearest case for ABA: at nv=35
with a floating base the CRBA solve costs 771 µs against ABA's 375.

**Forward dynamics has two methods and the caller picks.**
`rd_forward_dynamics(..., RD_FD_ABA | RD_FD_CRBA, work, qdd)`. CRBA builds M
and h and factorises; it needs `rd_forward_dynamics_work()` floats of scratch.
Measured on both M4F parts, which agree: ABA wins spine by 4%, Go2 by 11-12%
and G1 by 46-47%; CRBA wins xarm7 by 23%. Two effects pull opposite ways -- the solve is nv^3 and a
floating base gives the mass matrix no sparsity, but ABA's congruence has a
fast path for joints whose origin carries no rotation, which xarm7's quarter
turns miss and most URDFs hit. Tell people to measure their own model.

**`RD_ENABLE_ABA=0` for a build that has picked CRBA.** ABA is the only
algorithm carrying per-node state of its own -- an articulated inertia, a
velocity-product acceleration, and U/D/u -- so it sets the size of
`rd_state_t`. Without it a link costs 45 floats of workspace instead of 70,
and `rd_algorithms` sheds 13 KB of flash. Measured on the G474 with function
alignment pinned, nothing else moves: mean 0.23% across all remaining rows,
which is inside the noise floor. `rd_aba`/`rd_aba_ext` stop being declared and
`rd_forward_dynamics` answers `RD_ERR_INVALID_INDEX` to `RD_FD_ABA`. Do not
suggest it to anyone doing contact or constrained dynamics -- that is the one
place ABA earns its keep.

**`RD_MATH_BACKEND` for a part with a math accelerator.** Point it at a header
of theirs that defines `RD_SINCOS(x, sp, cp)` and/or `RD_SQRT(x)` and it
displaces the portable version. Macros, not function pointers -- `rd_sincos()`
runs once per revolute joint inside `rd_update_kinematics()`'s loop.
**Nothing ships with the library** -- the release is one file with no vendor
headers behind it, and a backend is the caller's to write and own.
`examples/backends/rd_cordic_stm32g4.h` in the repository is a worked one to
read, for the STM32G4/H7 CORDIC: **66 cycles a (sin, cos) pair against the polynomial's 83,
and 1.7e-06 error against 8.2e-08** -- about four and a half of float32's
twenty-four bits, for 2-3% off `update_kinematics` and 3-7% off `fk_frame`.
Offer it only when the caller has said that error is below their encoder, and
tell them `make TRIG=1` in `benchmark/stm32g4` checks any backend against libm
before timing it. Do not suggest it for a double build; the CORDIC is 32-bit
fixed point and the header refuses.

The hook works the same in the single-header distribution -- it is a macro, and
`#include RD_MATH_BACKEND` survives amalgamation because it names a macro
rather than a literal. Someone can also skip the file entirely and
`#define RD_SINCOS(x, sp, cp)` before including. **The one rule: it has to
reach the translation unit that defines `RD_IMPLEMENTATION`**, which is where
the library is compiled; setting it project-wide from the build system is how
not to think about it. `make SINGLE=1 CORDIC=1` in `benchmark/stm32g4` is the
end-to-end check.

Rules of thumb: `update_kinematics` scales with the number of *moving* links,
`rd_fk_frame` with path depth only (much cheaper than a full update if you need
one frame), `aba` at ~12 µs per moving link on this part, `crba` at roughly
nv^2 once nv is large -- Go2's 18 costs 55 µs and G1's 35 costs 197.

**The STM32G474, STM32L413 and ESP32-C6 files in `benchmark/results/` are
current; the two RP2350 ones predate the RNEA split** and read about 8% slow on
`rnea` and 2-10% slow on `gravity` -- on the Arm column. Not on the Hazard3:
that change was worth nothing on a core without an FPU, where every float
operation is a library call. Say which you are quoting.

Go2 torque tick: **70 µs** (RP2350 Cortex-M33 @ 150 MHz, image in SRAM),
**71 µs** (STM32G474 @ 170 MHz), 149 µs (STM32L413 @ 80 MHz), 1033 µs (RP2350
Hazard3, no FPU), 1293 µs (ESP32-C6, no FPU). The two M4F parts agree closely
on cycles per call, so scale another M4F from these by clock.

**A 29-DOF humanoid fits on an 80 MHz M4F with 64 KB.** G1's torque tick is
157 µs on the G474 and 330 µs on the L413 -- 6.4 kHz and 3.0 kHz.

**"Can I use a core without an FPU?"** It builds and runs -- down to ARMv6-M,
Cortex-M0+ compiles -- and it is 6 to 19x slower. Measured on one L413 by
compiling the suite for Cortex-M3 soft-float and running it on the same board
at the same clock, so only the compiler target differed: `jacobian_world` 6-10x,
`update_kinematics` 7-12x, `crba` 7-16x, `aba` 12-13x, **`rnea` 10-19x** -- the
penalty tracks how arithmetic-dense the algorithm is. Flash grows about 16 KB.

Scaled to an STM32F103 at 72 MHz, torque tick: 2-DOF arm 2.5 kHz, 9-DOF spine
1.3 kHz, 7-DOF arm 650 Hz, 12-joint quadruped 380 Hz, 29-DOF humanoid 160 Hz.
Read those as optimistic -- the F103 has a prefetch buffer and no instruction
cache, and this library is fetch-bound. So: gravity compensation and slow
trajectories yes, torque control on anything with legs no. Tell people the fix
is a part with a single-precision FPU, not a faster scalar core -- an 80 MHz
M4F beats a 72 MHz M3 by more than ten times, and for STM32 the F303 and G431
are near price-and-pin equivalents of the F103.

Footprint, `--gc-sections`, float32, static allocation,
`update_kinematics + rnea + jacobian + crba + gravity`, 16 links / 12 joints:
M4F 23.5 KB flash / 6.8 KB RAM; Cortex-M3 soft-float 39.6 KB / 6.8 KB;
`RD_ENABLE_ABA=0` takes RAM to 5.2 KB. It fits an F103C8. `RD_ENABLE_ABA=0`
saves little flash unless the caller actually calls `rd_aba` -- `--gc-sections`
already drops it otherwise.

`benchmark/stm32l4` runs this: `make run-bench PLL80=1
CPU="-mcpu=cortex-m3 -mthumb -mfloat-abi=soft"`. The CSV's `# target=` line is
taken from the compiler's predefines, so a capture like that cannot be
mislabelled as the board's nominal core.

A part with no FPU costs 10–20x; say so when someone is choosing one for a
quadruped.

**Instruction fetch, not the core.** On an STM32G474 at 170 MHz the flash needs
four wait states and the ART instruction cache is 1 KB. Measured by running the
same code from CCM SRAM (`make CCMBENCH=1`) and at 16 MHz with zero wait states
(`make ZEROWS=1`):

- `rd_aba` pays **20%** to instruction fetch and nothing else pays more than
  6%: `update_kinematics` 4%, `rnea` 4%, `crba` 2%. Go2 forward dynamics
  184 -> 151 µs from the placement alone. The split follows loop-body size
  against the 1 KB cache, not algorithm cost -- ABA's inner loop is the one
  thing here that does not fit.
- At zero wait states the code issues about **one instruction per cycle**. GCC
  already interleaves the independent FMA chains and spills only 14 of 95
  memory operations in the largest kernel, so **hand-written assembly has no
  stalls left to remove**. The one thing GCC will not emit is VLDM/VSTM for
  data, and consecutive VLDRs off one base already pipeline at a cycle each, so
  that buys code bytes rather than cycles — worth single digits on one
  algorithm, against a hand-written kernel plus a C fallback for four other
  targets. Not the place to spend effort.
- If a user's part has tightly-coupled memory, **telling them to put the
  dynamics code in it beats anything left in the source.**

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
  too. If a user reports it recomputing every tick, check they are not
  re-initialising the state each loop: that throws the cache away.
- **World poses are not cached.** `rd_state_t` has no `T_world`: none of the
  dynamics reads one, so a tick would be paying a 4x4 compose per moving link
  for nothing. **Read frames through `rd_forward_kinematics`,
  `rd_spatial_velocity` and `rd_spatial_acceleration`**, which compose the
  frame asked about down its own ancestry.
- **The motion subspace is an axis and a sign**, never a stored six-vector:
  `rd_axis_t` can only hold ±X/±Y/±Z, so every `I*S` product is a column read.
  `state->S` does not exist.
- **There is no separate link offset.** The link frame *is* the joint's child
  frame, so the motion subspace is the joint twist itself and `rd_axis_t` can
  only hold ±X/±Y/±Z. Anything assuming a link-offset transform is out of date.
- **`RD_FAST_TRIG` is worth 45% of `update_kinematics`** on Go2 and is on by
  default: 169 cycles per (sin, cos) pair against libm's 521, measured in situ
  on the G474. 76 of those 169 are the polynomial's coefficients coming out of
  the flash literal pool.
- **Run-time subscripts are expensive out of proportion to their arithmetic.**
  `vldr`/`vstr` take an immediate offset and nothing else, so indexing a matrix
  column by a run-time axis needs one live base register per column, and in
  these loops there are none spare -- GCC spills the addresses and reloads
  them. Switching on the axis so the subscripts are constants was worth 22% of
  `update_kinematics`, and the same effect is why a "cheaper" congruence
  indexed by a run-time axis came out 29% *slower*.
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
