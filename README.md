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
| `update_kinematics` | 6.71 | 10.36 | 17.92 | 27.35 |
| `fk_frame` | 7.20 | 9.56 | 18.28 | 10.62 |
| `jacobian_world` | 6.31 | 10.68 | 15.06 | 12.31 |
| `jacobian_local` | 8.75 | 16.90 | 21.96 | 19.93 |
| `rnea` | 14.06 | 18.26 | 32.29 | 51.45 |
| `aba` | n/a | 57.41 | 104.06 | 180.96 |
| `fd_crba` | n/a | 55.46 | 84.52 | 197.64 |
| `crba` | 10.34 | 18.64 | 40.52 | 60.88 |
| `gravity` | 9.82 | 12.65 | 22.06 | 34.52 |
| `spatial_acceleration` | 8.40 | 10.37 | 18.70 | 22.84 |
| `spatial_velocity` | 3.11 | 3.78 | 7.03 | 4.71 |

`simple_arm`'s URDF carries no inertial data, so `rd_aba` correctly refuses it
(`RD_ERR_SINGULAR`) and its dynamics columns are traversal cost only.

What that buys per control tick on the same part:

| Robot | dof | Torque tick | Operational space | Forward dynamics |
|---|---|---|---|---|
| `spine` | 9 | 29 µs — 34.9 kHz | 47 µs — 21.2 kHz | 66 µs — 15.2 kHz |
| `xarm7` | 7 | 50 µs — 19.9 kHz | 91 µs — 11.0 kHz | 102 µs — 9.8 kHz |
| `go2` | 18 | 79 µs — 12.7 kHz | 140 µs — 7.2 kHz | 208 µs — 4.8 kHz |

Torque tick is `update_kinematics` + `rnea`; operational space adds `crba`;
forward dynamics is `update_kinematics` + whichever of the two methods below is
faster for that robot.

Go2 (18 DOF, 31 links) across the five cores measured so far, µs per call:

| | M4F @ 170<br><sub>G474</sub> | M4F @ 80<br><sub>L413</sub> | M33 @ 150<br><sub>RP2350</sub> | Hazard3 @ 150<br><sub>RP2350</sub> | RV32 @ 160<br><sub>ESP32-C6</sub> |
|---|---|---|---|---|---|
| | **FPU** | **FPU** | **FPU** | *no FPU* | *no FPU* |
| `update_kinematics` | 27.4 | **56.8** | 155.2 | 1458.8 | 354.5 |
| `rnea` | 51.5 | **100.6** | 125.0 | 1650.3 | 939.4 |
| `crba` | 60.9 | **126.1** | 234.4 | 2293.0 | 848.9 |
| `aba` | 181.0 | **372.1** | 399.6 | 4239.1 | 2726.6 |
| torque tick | 79 µs<br>12.7 kHz | **157 µs<br>6.4 kHz** | 280 µs<br>3.6 kHz | 3109 µs<br>322 Hz | 1294 µs<br>773 Hz |
| operational space | 140 µs<br>7.2 kHz | **283 µs<br>3.5 kHz** | 515 µs<br>1.9 kHz | 5402 µs<br>185 Hz | 2143 µs<br>467 Hz |

> The STM32L413 column is the newest. The G474 and ESP32-C6 columns predate the
> last change, which is worth about 3% on a torque tick and 12% on
> `spatial_acceleration`; they will be refreshed when those boards are back on
> the desk. The two RP2350 columns are the properly stale ones, 2–3x behind.

The two Cortex-M4F parts agree to **within 2.5% on cycles per call** across
every algorithm (median 0.975), so **another M4F can be scaled from these by
clock** and be close. What this round bought those two boards, on Go2:

| | STM32L413 @ 80 MHz | ESP32-C6 @ 160 MHz |
|---|---|---|
| `update_kinematics` | 489.5 → 56.9 µs — 8.6x | 1495.0 → 354.5 µs — 4.2x |
| `crba` | 971.5 → 126.1 µs — 7.7x | 3627.7 → 848.9 µs — 4.3x |
| `aba` | 1299.5 → 378.5 µs — 3.4x | 6243.3 → 2726.6 µs — 2.3x |
| torque tick | 567 → 163 µs — 3.5x | 3819 → 1294 µs — 3.0x |
| operational space | 1732 → 289 µs — 6.0x | 7447 → 2143 µs — 3.5x |

### Two forward dynamics, and which to pick

`rd_forward_dynamics()` takes a method. Both give the same `qdd` -- they agree
to 3.2e-13 in double precision -- and which is faster is a property of the
robot rather than of the library, so it is your choice rather than a heuristic:

| | recursion | workspace | scales as |
|---|---|---|---|
| `RD_FD_ABA` | articulated-body | none beyond `rd_state_t` | O(n) |
| `RD_FD_CRBA` | M(q), h(q,qd), then Cholesky | `nv*nv + nv` floats | O(n³) in the solve |

Measured on the STM32G474, `update_kinematics` + the method:

| Robot | nv | ABA | CRBA | |
|---|---|---|---|---|
| `xarm7` | 7, fixed base | 122.0 µs | **102.4 µs** | CRBA −16% |
| `spine` | 9, floating base | 67.8 µs | **65.8 µs** | CRBA −3% |
| `go2` | 18, floating base | **208.3 µs** | 225.0 µs | ABA −8% |

The same ordering holds on the STM32L413 (−17% / −3% / +8%) and on the
FPU-less ESP32-C6 (−13% / −13% / +3%), so the crossover is a property of the
robot rather than of the core.

The crossover is around ten to twelve velocity DOF, and a floating base pushes
it down: its six DOF are ancestors of every joint, so the mass matrix has no
sparsity for the factorisation to exploit. The CRBA route also hands back M
and h, which an operational-space controller wants anyway. The benchmark's
`fd_crba` row against `aba` is exactly this comparison, so you can run it on
your own model.

### How it gets this fast

Nothing here is a trick, and nothing trades accuracy for speed — every number
above passes the same Pinocchio comparison. The gains come from noticing what
the general form of an algorithm is spending time rediscovering, and from
measuring on the actual part rather than reasoning about it.

**Say less, and the arithmetic collapses.** `rd_axis_t` can only hold ±X, ±Y or
±Z, and the link frame *is* the joint's child frame. So the motion subspace is
always a unit spatial axis. Every `I·S` becomes a column read instead of a
matrix-vector product; every `v × S` is four multiplies instead of 24; every
Jacobian column is nine instead of 24. A joint's own transform is never built
at all — composing straight through its parent's offset is 12 multiplies where
forming the 4×4 and running a general SE(3) multiply is 36 plus 20 stores. None
of this needs a test at run time, because the type cannot express anything else.

**Store what the algorithm actually is.** A sum of rigid-body inertias is a
rigid body, so CRBA's composite lives in ten numbers, not a 6×6. ABA's
articulated inertia stops being a rigid body after the rank-1 downdate but
stays symmetric, so 21 numbers, not 36. Both exact.

**A real robot is mostly frames that cannot move** — feet, sensor mounts,
inertial frames; Go2 is 18 of 31 links. Each one's inertia is folded into its
nearest moving ancestor when the chain is built, and the dynamics traverse only
the moving nodes. World poses are not cached at all: none of RNEA, CRBA or ABA
reads one, so a control tick was paying a 4×4 compose per link for the frame
queries' benefit. Those compose the frames they are asked about instead.

**Write only what changed.** For a moving link whose parent also moves, ten of
its 4×4's sixteen floats do not depend on the joint angle — the axis column
passes through, the translation is the offset's, the bottom row is the bottom
row. They are written once, when the state first meets the chain.

**Nothing allocates, ever.** One caller-provided buffer holds the whole tick:
85 floats per link, and no `malloc` anywhere in the library.

Then there is a category that only shows up on the hardware:

**Run-time subscripts cost far more than their arithmetic.** `vldr` and `vstr`
take an immediate offset and nothing else, so indexing a matrix column by a
run-time axis needs one live base register per column touched — and in these
loops there are none spare, so the compiler computes the addresses, spills them
and reloads each one before its float. Switching on the axis so the subscripts
are compile-time constants removed 17 integer instructions per link and was
worth 22% of `update_kinematics`. The same effect explains a failure: a
congruence rewritten to use 24 multiplies instead of 45, indexed by a run-time
axis, came out **29% slower**. Forty-five multiplies in registers beat 24
through memory.

**libc is not free.** Every algorithm clears its output first, and the size is
only known at run time, so the compiler cannot inline the clear and calls
whatever libc is linked. newlib-nano's `memset` — what `--specs=nano.specs`
selects, and what STM32CubeIDE ships by default — is a byte-at-a-time loop.
Clearing Go2's 18×18 mass matrix through it cost 9,039 cycles, **half of
CRBA**. A typed store loop does not care which libc the caller links.

**The compiler is usually right, and it is worth checking which times it is
not.** At zero flash wait states this code issues about one instruction per
cycle: GCC already interleaves the independent FMA chains and spills 14 of 95
memory operations in the largest kernel, so there are no pipeline stalls left
for hand-written assembly to remove. Roughly a third of the ideas tried this
way lost — global `-O2` (38% slower), linking to RAM, a branch to skip
redundant work (15% slower), rolling the Cholesky onto pointers. They were
reverted, and the ones that stayed were kept because a board said so.

### Against code generation

The fastest thing a general-purpose library can hand you is not its recursion
but the code it generates for your robot. Pinocchio traces RNEA, ABA and CRBA
symbolically and CasADi emits the straight-line C: no loops, no model
traversal, every operation for one specific robot spelled out, common
subexpressions eliminated. Go2's ABA comes out as 5,773 operations and 122 KB
of C. That is the right thing to measure against, so
[`benchmark/codegen/`](benchmark/codegen/) does.

Both sides are float32, both `-O3`, same compiler, same board, and both are
checked against Pinocchio's own double-precision answer rather than against
each other — they agree with it to about 1e-7, which is all float32 has.

**On a desktop, code generation wins.** Go2, nanoseconds per call on an
x86-64 host: RNEA 207 against 184, ABA 579 against 413, CRBA 232 against 140.
An out-of-order core with register renaming and a 32 KB instruction cache is
exactly what fully unrolled straight-line code is built for.

**On the microcontroller it inverts.** Go2 on an STM32L413 at 80 MHz, cycles
per call, `update_kinematics` + the algorithm against one generated call:

| | RobotDynamics | code generation | |
|---|---|---|---|
| `rnea` | **13,161** | 50,729 | 3.9x |
| `aba` | **35,042** | 72,215 | 2.1x |
| `crba` | **14,696** | 44,823 | 3.1x |
| `rnea` + `crba` | **23,239** | 95,534 | 4.1x |

Two things account for it. A Cortex-M4 is in-order with 32 FP registers and no
renaming, so several thousand simultaneously-live temporaries become spills to
stack, and each one is a real load and a real store. And the code is 66,534
bytes for Go2's three algorithms, against 24,794 bytes for the *whole library*
— running the same binary at 16 MHz, where the flash needs no wait states,
shows the generated code paying a **25–29%** instruction-fetch tax where our
RNEA and CRBA pay 7–9%. The exception proves the rule: our ABA pays 30% too,
because its inner loop is the one thing here that does not fit the 1 KB
instruction cache.

Code generation trades code size for arithmetic. On a desktop that is a good
trade. One robot's worth of it is 2.7x the size of this entire library, which
handles any robot.

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
