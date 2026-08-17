# Profiling RobotDynamics on the RP2350

Measured performance of every algorithm in the library, on both CPU
architectures of a Raspberry Pi Pico 2, across four robot platforms.

All raw data is in [`benchmark/results/`](../benchmark/results/); every table
below is generated from it by [`tools/report.py`](../tools/report.py). For the
correctness side, see [VALIDATION.md](VALIDATION.md) — every algorithm measured
here agrees with Pinocchio to machine precision.

---

## Summary

1. **The Arm cores are 4–13x faster than the RISC-V cores** on this workload
   (median 9.7x across 39 floating-point measurements) — not because Hazard3 is
   a weak core, but because **the RP2350's RISC-V cores have no FPU**. Every
   `float` operation is emulated in software. An integer-only control
   measurement in the same run shows RISC-V up to 1.4x *faster*, which isolates
   floating point as the entire cause.

2. **On the Arm cores the library is comfortably real-time.** A torque control
   tick (`update_kinematics` + `rnea`) costs 326 µs for an 18-DOF Go2 and 47 µs
   for the 9-DOF spine — 3.1 kHz and 21.5 kHz. On the RISC-V cores the same Go2
   tick costs 3.5 ms, i.e. 286 Hz, well under a 1 kHz torque loop.

3. **Two structural rewrites bought 1.5–2.0x on the dynamics.** Exploiting the
   block structure of the spatial-inertia congruence halved CRBA (2.05x on Go2)
   and cut a third off ABA (1.53x); specialising the SE(3) compose and the
   coordinate-axis rotation cut `update_kinematics` by another 1.46x and
   standalone FK by 1.66x. A Go2 operational-space tick went from 1.1 kHz to
   1.7 kHz, and forward dynamics from 1.0 kHz to 1.6 kHz. See
   [Optimisation history](#optimisation-history).

4. **ABA scales almost perfectly linearly** — 17.5, 16.4 and 15.7 µs per link on
   the three platforms that have inertial data, across a 4-link chain and a
   31-link branched tree.

5. **Soft-float execution time is data-dependent.** An operation that does
   provably identical work spans 8.29–13.65 µs on RISC-V depending on the
   *values* flowing through it, while staying flat at 1.06–1.07 µs on Arm. That
   matters if you need a worst-case execution time bound.

---

## Hardware and toolchain

The RP2350 is unusually well suited to an ISA comparison: it carries **two
Cortex-M33 cores and two Hazard3 RISC-V cores on the same die**, sharing one
clock tree, one bus fabric and one set of memories. Switching between them
changes the CPU and nothing else.

| | Arm | RISC-V |
|---|---|---|
| Core | Cortex-M33 | Hazard3 |
| `-march` | `armv8-m.main+fp+dsp` | `rv32imac_zicsr_zifencei_zba_zbb_zbs_zbkb` |
| Float ABI | `softfp` (**hardware** FPv5-SP instructions) | `ilp32` (**software** emulation) |
| FPU | yes, single precision | **none** |
| Clock | 150 MHz | 150 MHz |
| Compiler | `arm-none-eabi-gcc` 15.2.1 | `riscv32-pico-elf-gcc` 16.1.0 |
| Optimisation | `-O3` (identical) | `-O3` (identical) |
| `.text` | 80,884 B | 109,408 B |

> `-mfloat-abi=softfp` on the Arm side refers only to the *calling convention*
> (FP arguments travel in integer registers). Arithmetic still uses hardware
> VFP instructions — 2,160 of them appear in the linked image. The RISC-V image
> instead links the soft-float helpers `__addsf3`, `__subsf3`, `__mulsf3`,
> `__divsf3`, `__floatsisf` and `__fixsfsi`. That difference accounts for the
> 35% larger RISC-V binary.

Board: Raspberry Pi Pico 2 (RP2350A), Pico SDK 2.3.0, `PICO_PLATFORM` set to
`rp2350-arm-s` and `rp2350-riscv`.

---

## Robot platforms

The same robots the reference PyTorch implementation
([bard](https://github.com/YueWang996/bard-pytorch-dynamics)) uses, converted
from the identical URDFs by [`tools/urdf2c.py`](../tools/urdf2c.py):

| Platform | Links | Joints | nq | nv | Base | Tree shape | Query frame (path length) |
|---|---|---|---|---|---|---|---|
| `simple_arm` | 3 | 2 | 2 | 2 | fixed | serial | `end_effector_link` (3) |
| `spine` | 4 | 3 | 10 | 9 | floating | serial | `front_body` (4) |
| `xarm7` | 10 | 7 | 7 | 7 | fixed | serial | `link_eef` (10) |
| `go2` | 31 | 12 | 19 | 18 | floating | 4 branches | `FL_foot` (5) |

`fk_frame`, both Jacobian rows and the spatial-quantity rows are measured
against the query frame, chosen as the deepest node in each tree.

Go2 keeps all 31 URDF links, including the 12 fixed-joint rotor bodies. They
carry inertia and are traversed like any other link, so the numbers reflect the
URDF as written rather than a fixed-joint-merged variant.

The floating-base platforms are driven with a non-identity base pose, a non-zero
base twist and a non-zero base acceleration, so these are the costs of the real
floating-base path rather than of a base pinned at the origin.

**`simple_arm` has no inertial data.** Its URDF declares no `<inertial>` block on
any link, so every mass and inertia is zero. `rd_aba` correctly refuses it with
`RD_ERR_SINGULAR` — a massless moving link has no articulated inertia to invert —
and the harness reports that rather than timing the error return. Its RNEA, CRBA
and gravity rows do run, but they are traversal cost over a zero inertia and are
not physically meaningful; read that column as a kinematics data point only.

---

## Methodology

* Each algorithm runs in a loop whose length is auto-calibrated to take at
  least 50 ms, so the 1 µs timer tick never limits resolution.
* That loop is then run **5 times and the minimum is kept**. The minimum
  rejects interference from the USB interrupt servicing stdio in the
  background.
* Timing uses the RP2350's 1 MHz timer, read identically from either core —
  deliberately *not* a core cycle counter, since Arm's `DWT->CYCCNT` and
  Hazard3's `mcycle` CSR are different mechanisms with different read costs.
* Every algorithm folds part of its output into a checksum that is printed at
  the end, so the optimiser cannot delete the calls.
* Firmware is linked `copy_to_ram`. Running from flash would make results
  sensitive to XIP cache hit rate and to each toolchain's code layout, which
  would confound the comparison.
* Joint and base states come from a fixed xorshift32 seed, so both
  architectures see bit-identical inputs. Angles are kept away from zero so no
  trigonometric fast path is taken.
* The algorithms follow the library's intended pattern: one
  `rd_update_kinematics` primes `rd_state_t`, then each algorithm reuses it.
  That priming call is timed separately as its own line item.
* Every algorithm is run once and its status checked before being timed. One
  that bails out early would otherwise be "measured" at the cost of its error
  return, which is worse than reporting nothing.

**Repeatability.** The Arm suite was flashed and run twice from scratch. Across
all measurements the largest deviation between runs was **below 0.005%**;
most were bit-identical. The workload is deterministic and the board has no
frequency scaling, so run-to-run noise is not a factor in anything below.

---

## Results — Arm (Cortex-M33 @ 150 MHz)

Microseconds per call. Multiply by 150 for cycles.

| Algorithm | simple_arm<br><sub>3L / 2 dof</sub> | spine<br><sub>4L / 9 dof</sub> | xarm7<br><sub>10L / 7 dof</sub> | go2<br><sub>31L / 18 dof</sub> |
|---|---|---|---|---|
| `update_kinematics` | 16.13 | 23.06 | 54.93 | 153.53 |
| `fk_frame` | 9.65 | 13.19 | 29.44 | 14.65 |
| `jacobian_world` | 3.04 | 8.31 | 7.92 | 9.23 |
| `jacobian_local` | 4.36 | 13.05 | 11.68 | 18.47 |
| `rnea` | 17.37 | 23.41 | 57.17 | 172.05 |
| `aba` | n/a | 69.85 | 163.76 | 487.21 |
| `crba` | 20.21 | 33.45 | 98.79 | 246.69 |
| `gravity` | 11.43 | 15.22 | 36.32 | 107.23 |
| `spatial_accel` | 6.98 | 9.25 | 21.75 | 65.52 |
| `spatial_velocity` | 1.06 | 1.07 | 1.07 | 1.06 |
| *`_heap_probe`* (control) | 6.51 | 7.03 | 7.87 | 11.70 |

## Results — RISC-V (Hazard3 @ 150 MHz)

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 125.98 | 224.48 | 529.96 | 1458.33 |
| `fk_frame` | 87.29 | 154.00 | 336.31 | 170.86 |
| `jacobian_world` | 12.61 | 60.55 | 50.61 | 60.81 |
| `jacobian_local` | 29.09 | 163.17 | 138.51 | 208.53 |
| `rnea` | 120.45 | 272.10 | 674.91 | 2041.28 |
| `aba` | n/a | 720.26 | 1805.78 | 4648.78 |
| `crba` | 137.48 | 322.82 | 1235.17 | 2291.06 |
| `gravity` | 60.86 | 105.27 | 282.19 | 791.81 |
| `spatial_accel` | 49.68 | 105.91 | 244.74 | 726.52 |
| `spatial_velocity` | 8.29 | 13.65 | 13.66 | 13.64 |
| *`_heap_probe`* (control) | 5.85 | 6.30 | 6.66 | 8.51 |

## RISC-V ÷ Arm

Higher means RISC-V is slower.

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 7.81x | 9.73x | 9.65x | 9.50x |
| `fk_frame` | 9.05x | 11.68x | 11.42x | 11.66x |
| `jacobian_world` | 4.15x | 7.28x | 6.39x | 6.59x |
| `jacobian_local` | 6.67x | 12.50x | 11.86x | 11.29x |
| `rnea` | 6.94x | 11.62x | 11.81x | 11.86x |
| `aba` | -- | 10.31x | 11.03x | 9.54x |
| `crba` | 6.80x | 9.65x | 12.50x | 9.29x |
| `gravity` | 5.32x | 6.92x | 7.77x | 7.38x |
| `spatial_accel` | 7.12x | 11.45x | 11.25x | 11.09x |
| `spatial_velocity` | 7.82x | 12.76x | 12.77x | 12.87x |
| **`_heap_probe`** (integer only) | **0.90x** | **0.90x** | **0.85x** | **0.73x** |

The last row is the control. It exercises `malloc`/`free` — pointer arithmetic
and free-list walking, no floating point at all — and RISC-V wins every one of
them, running the allocator up to 1.37x faster than the M33. Hazard3 is not a
slow core. It simply has no FPU, and this library is almost entirely
floating-point arithmetic. (The library itself no longer allocates anywhere;
this probe exists only as an architecture comparison that touches no floats.)

---

## Where the time goes

### The 6x6 inertia congruence still dominates CRBA and ABA

| Robot | dof | RNEA | ABA | ABA/RNEA | CRBA | ABA/CRBA | ABA per link |
|---|---|---|---|---|---|---|---|
| `spine` | 9 | 23.41 µs | 69.85 µs | 2.98x | 33.45 µs | 2.09x | 17.5 µs |
| `xarm7` | 7 | 57.17 µs | 163.76 µs | 2.86x | 98.79 µs | 1.66x | 16.4 µs |
| `go2` | 18 | 172.05 µs | 487.21 µs | 2.83x | 246.69 µs | 1.97x | 15.7 µs |

`algo_transform_inertia_accumulate` runs once per link and evaluates `XᵀIX`.
Rewriting it against the block structure (see
[Optimisation history](#optimisation-history)) took CRBA down by half, but it is
still the single largest cost in both algorithms.

ABA's per-link cost is nearly flat across a 4-link serial chain, a 10-link arm
and a 31-link branched tree — 17.5, 16.4, 15.7 µs — which confirms that this one
function, not the tree walking around it, is where the time goes.

**On ABA vs CRBA.** ABA being ~2x CRBA is *not* by itself evidence of a problem,
and an earlier version of this document overstated that. The fair comparison for
forward dynamics is ABA against CRBA **plus** the nonlinear terms **plus** an
nv x nv factorisation: for Go2 that is 246.69 + ~172 = 419 µs before the solve,
against ABA's 487 µs. The two routes are close at this size, which is roughly
where the literature puts the crossover; ABA's advantage is asymptotic in nv,
since the competing route carries an O(nv³) factorisation.

### `fk_frame` scales with depth, not link count

Go2 has three times the links of xarm7 but its standalone FK is **twice as
fast**. `rd_fk_frame` walks only the root-to-frame path, and Go2's tree is four
shallow branches of depth 5 while xarm7 is a single serial chain of depth 10.

Sorted by path length rather than link count, the cost is almost exactly linear
at about 5 µs per level on Arm:

| Platform | Links | Path length | `fk_frame` | µs per level |
|---|---|---|---|---|
| `simple_arm` | 3 | 3 | 9.65 µs | 3.22 |
| `spine` | 4 | 4 | 13.19 µs | 3.30 |
| `go2` | 31 | 5 | 14.65 µs | 2.93 |
| `xarm7` | 10 | 10 | 29.44 µs | 2.94 |

If you only need one frame's pose, this is much cheaper than a full
`update_kinematics` — 15 µs against 154 µs on Go2.

### `update_kinematics` is the largest single kinematics item

It is the only algorithm that touches every link unconditionally, and the only
one that calls `sinf`/`cosf` — once per actuated joint. At 154 µs for Go2 it is
now slightly cheaper than RNEA. It is called once per control tick regardless of
how many algorithms follow, which is the entire point of the design.

### `rd_gravity()` is now genuinely cheaper than RNEA

It runs the same recursion with the cached velocities suppressed, which drops
the Coriolis cross-products and the `I v` bias force. That shows up as a
consistent **0.62–0.66x** of full RNEA across all four platforms. Before v0.2.0
this entry point was the same call as `rd_nonlinear_terms()` and cost exactly
the same — the speedup is the semantic fix becoming visible.

### What removing the allocator bought

v0.1.0 called `malloc`/`free` inside RNEA and CRBA on every invocation. Moving
that scratch into `rd_state_t` for v0.2.0 shows up where you would expect —
on the small models, where the allocator was a large fraction of a short call:

| Robot | RNEA, v0.1.0 | RNEA, v0.2.0 | change |
|---|---|---|---|
| simple_arm | 20.53 µs | 17.33 µs | −16% |
| spine | 26.79 µs | 23.44 µs | −13% |
| xarm7 | 58.65 µs | 56.99 µs | −3% |
| go2 | 170.27 µs | 171.47 µs | +1% |

On Go2 it is a wash: the allocator was only 5.6% of that call, and the scratch
now sits further away in memory. The reason to do it was never mainly speed —
`malloc` in a control loop may take a lock and can fragment over a long run, and
ABA's much larger workspace would have made the per-call allocation worse.

### Soft-float timing is data-dependent

`rd_spatial_velocity` does the same fixed amount of work for every robot: one
memcpy and one 6-vector spatial transform. Its cost confirms that on Arm —
**1.06, 1.07, 1.07, 1.06 µs** across the four platforms, flat to within the
timer's resolution.

On RISC-V the same call takes **8.29, 13.65, 13.66, 13.64 µs** — a 1.65x spread
for identical work, driven purely by the operand values. The soft-float helpers
branch on zeros, exponent alignment and normalisation shifts, so execution time
varies with the numbers themselves. If you need a worst-case execution time
bound on the RISC-V cores, you cannot get it from a single measurement the way
you can on Arm.

---

## Control-loop budgets

A torque control tick of `update_kinematics` + `rnea`:

| Robot | dof | Arm | max rate | RISC-V | max rate |
|---|---|---|---|---|---|
| `simple_arm` | 2 | 33.5 µs | 29.9 kHz | 246 µs | 4.1 kHz |
| `spine` | 9 | 46.5 µs | 21.5 kHz | 497 µs | 2.0 kHz |
| `xarm7` | 7 | 112.1 µs | 8.9 kHz | 1205 µs | 830 Hz |
| `go2` | 18 | 325.6 µs | 3.1 kHz | 3500 µs | 286 Hz |

Adding CRBA, for operational-space or inverse-dynamics control that needs the
mass matrix, on Arm: spine 79.9 µs (12.5 kHz), xarm7 210.9 µs (4.7 kHz), Go2
572.3 µs (1.7 kHz).

A **simulation** tick instead — `update_kinematics` + `aba`:

| Robot | dof | Arm | max rate |
|---|---|---|---|
| `spine` | 9 | 92.9 µs | 10.8 kHz |
| `xarm7` | 7 | 218.7 µs | 4.6 kHz |
| `go2` | 18 | 640.7 µs | 1.6 kHz |

Go2 forward dynamics runs at 1.6 kHz on one Arm core, up from 1.0 kHz before the
optimisation work below.

**Practical reading.** On the Arm cores every platform here clears a 1 kHz
torque loop with margin, and Go2 clears it even with CRBA in the loop — barely.
On the RISC-V cores only the two smallest robots clear 1 kHz. Unless you need
the RISC-V cores for another reason, **run dynamics on the Arm cores**.

---

## Host reference

Same code, same models, Apple clang `-O2` on an Apple M4 Pro, for scale.
Microseconds per call:

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 0.04 | 0.05 | 0.13 | 0.35 |
| `fk_frame` | 0.03 | 0.04 | 0.08 | 0.04 |
| `rnea` | 0.04 | 0.06 | 0.16 | 0.44 |
| `aba` | n/a | 0.27 | 0.58 | 1.42 |
| `crba` | 0.05 | 0.10 | 0.30 | 0.74 |

Roughly 290–460x the Arm core's throughput, about what the clock ratio and a
wide out-of-order pipeline predict. Useful mostly as a sanity check that the
algorithms are not pathologically slow.

---

## Optimisation history

Two rewrites, each measured on the board in isolation so the attribution is
clean. Both were validated against Pinocchio before and after: agreement stayed
at machine precision throughout, so nothing here trades accuracy for speed.

### Round 1 — structure in the spatial-inertia congruence

`XᵀIX` was two dense 6x6 products: 432 multiplies and 360 adds per link, plus
building the adjoint, transposing it, and four 36-float temporaries on a 2 KiB
stack. But `X = Ad(T)` carries a 3x3 zero block, `I` is symmetric, and so is the
result. Expanding blockwise with `I = [[A11, A12], [A21, A22]]`:

```
TL = Rᵀ A11 R
TR = Rᵀ (A11 P + A12) R                        P = [p]x
BL = TRᵀ                                       free, was being recomputed
BR = Rᵀ (A22 + A21 P + (A21 P)ᵀ - P A11 P) R
```

Multiplying by a skew matrix is a cross product, 18 multiplies rather than 27.
The whole thing costs 216 multiplies instead of 432, with no 6x6 temporary.

### Round 2 — structure in the transforms

`rd_mat4_mul` was a general 4x4 product, but every matrix the library composes
is a rigid motion whose bottom row is `[0 0 0 1]`; a real SE(3) compose is 36
multiplies instead of 64. And `rd_rot_axis_angle` normalised the axis with a
sqrt and a divide before evaluating Rodrigues' formula — for axes that
`rd_axis_t` already restricts to ±X, ±Y, ±Z, where the answer is just `sincos`
placed into a fixed pattern. Both now take the specialised path, with the
general form kept as a fallback so a hand-written model cannot be silently
mis-rotated.

### Measured, Arm Cortex-M33

Microseconds per call. "before" is v0.2.0.

| Robot | Algorithm | before | round 1 | round 2 | total |
|---|---|---|---|---|---|
| `go2` | `crba` | 505.24 | 246.41 | 246.69 | **2.05x** |
| `go2` | `aba` | 746.57 | 488.62 | 487.21 | **1.53x** |
| `go2` | `update_kinematics` | 224.14 | 223.04 | 153.53 | **1.46x** |
| `go2` | `fk_frame` | 24.37 | 24.51 | 14.65 | **1.66x** |
| `go2` | `rnea` | 171.47 | 172.09 | 172.05 | 1.00x |
| `go2` | `gravity` | 106.54 | 106.91 | 107.23 | 0.99x |

Round 1 moved only the two algorithms that use the congruence; round 2 moved
only the two that build transforms. Nothing else shifted by more than 0.4%,
which is the harness's noise floor.

### What it buys a control loop, Go2 on one Arm core

| Tick | before | after | |
|---|---|---|---|
| torque, `update_kinematics` + `rnea` | 395.6 µs — 2.5 kHz | 325.6 µs — 3.1 kHz | 1.22x |
| operational space, + `crba` | 900.9 µs — 1.1 kHz | 572.3 µs — 1.7 kHz | **1.57x** |
| forward dynamics, `update_kinematics` + `aba` | 970.7 µs — 1.0 kHz | 640.7 µs — 1.6 kHz | **1.51x** |

The RISC-V cores see the same wins slightly amplified — 1.71x on CRBA, 1.39x on
`update_kinematics` for Go2 — because with no FPU every multiply removed is a
function call removed.

---

## Known issues

Properties of the library as it currently stands, surfaced by building,
measuring and validating it.

### `RD_USE_STATIC_ALLOC=1` is still unsupported

Under that flag `RD_MALLOC`/`RD_CALLOC` expand to `NULL`. As of v0.2.0 no
algorithm allocates, but `rd_chain_build()` still does, once, at startup, so it
returns `RD_ERR_ALLOC_FAILED`. The control loop is allocation-free either way;
what remains is giving `rd_chain_build` a caller-provided arena too.

### Topological ordering relies on an undocumented invariant

`rd_chain_build` builds `topo_order` with a hand-unrolled BFS that only reaches
depth 3. Anything deeper falls through to a fallback that appends the remaining
nodes **in link-index order**. That is a valid topological order only because
every model declares parents before children. A model with a child at a lower
index than its parent would silently produce a wrong ordering rather than an
error. `tools/urdf2c.py` guarantees the invariant and
`tools/test_urdf2c.py::test_parent_precedes_child` asserts it; a hand-written
model could still violate it.

### Fixed along the way

* **CRBA transformed composite inertia in the wrong direction** — mass matrix
  was 13–40% wrong. See [VALIDATION.md](VALIDATION.md).
* **ABA folded the velocity-product term in after the joint solve** instead of
  before it, making every joint acceleration wrong while leaving the base
  correct.
* **`rd_update_kinematics` ignored the floating base**, forcing the root to the
  identity pose with zero velocity, and RNEA double-rotated gravity.
* **`rd_gravity_compensation()` returned `C(q,q̇)q̇ + g(q)`.** Replaced by
  `rd_gravity()`, which runs the RNEA recursion with the cached velocities
  suppressed and so needs no second kinematics pass.
* **RNEA and CRBA allocated scratch on every call.** All scratch now lives in
  `rd_state_t`.
* **`tools/urdf2c.py` emitted joints breadth-first**, which scrambled `q`
  against Pinocchio and bard on branched robots.
* **`RD_USE_SINGLE_PRECISION=0` did not give double precision** — `rd_math.h`
  hardcoded the `float` libm variants.
* **The repository did not compile**: `robot_dynamics.h` never included
  `rd_algorithms.h`, `test_main.c` targeted a removed API, and the top-level
  `install()` list named four headers that do not exist.

---

## Ranked optimisation opportunities

Done so far: scratch moved into `rd_state_t` (v0.2.0), CRBA's duplicate
parent-chain walk removed (v0.2.0), the inertia congruence and the SE(3) /
coordinate-axis transforms specialised (v0.3.0). What is left, in rough order of
expected value:

1. **Exploit symmetry in the congruence output.** `Rᵀ A11 R` and the bottom-right
   block are symmetric, but `algo_congruence3` still computes all nine entries of
   each. Six would do. Perhaps another 15% off CRBA and ABA.

2. **Exploit that `S` has exactly one non-zero entry.** Because joint axes are
   restricted to ±X/±Y/±Z, the joint subspace `S` is a unit basis vector, so
   `rd_mat6_vec(I, S, U)` is a column extract and `S · f` is a single element.
   RNEA, CRBA and ABA all currently do full 6-vector arithmetic there. RNEA is
   the one algorithm neither round touched, and this is its main opportunity.

3. **Do ABA's rank-1 downdate in place.** `rd_aba` copies a 36-float articulated
   inertia per link before modifying it; the original is not needed afterwards.

4. **Cache `sincos` per joint in `rd_state_t`.** `update_kinematics` calls it
   once per joint and `rd_fk_frame` then recomputes the same values.

5. **Let the compiler fuse multiply-adds.** The benchmark builds with `-O3` but
   no `-ffp-contract=fast`, so the M33's `VFMA` goes unused in code that is
   almost entirely dot products.

6. **Run dynamics on the Arm cores.** Nothing in the library needs changing —
   this is a deployment decision, and it is worth 4–13x.

7. **If the RISC-V cores are unavoidable, consider fixed point.** With no FPU, a
   Q-format implementation would very likely beat soft float, and would restore
   deterministic timing.

---

## Reproducing

```bash
# Host
cmake -B build && cmake --build build && ./build/rd_test

# Pico 2, Arm cores
cd benchmark
PICO_SDK_PATH=/path/to/pico-sdk cmake -B build-arm -G Ninja -DPICO_PLATFORM=rp2350-arm-s
cmake --build build-arm
python3 ../tools/capture.py build-arm/rd_benchmark.uf2 results/rp2350_arm.csv

# Pico 2, RISC-V cores (needs the riscv-toolchain from raspberrypi/pico-sdk-tools)
PICO_SDK_PATH=/path/to/pico-sdk PICO_TOOLCHAIN_PATH=/path/to/riscv \
  cmake -B build-riscv -G Ninja -DPICO_PLATFORM=rp2350-riscv
cmake --build build-riscv
python3 ../tools/capture.py build-riscv/rd_benchmark.uf2 results/rp2350_riscv.csv

# Regenerate the tables in this document
python3 tools/report.py benchmark/results/*.csv
```

`capture.py` reboots a running board into BOOTSEL, flashes the image, waits for
the USB CDC device to re-enumerate and scrapes the CSV report off it.

See [`benchmark/README.md`](../benchmark/README.md) for the full setup,
including where to get each toolchain.
