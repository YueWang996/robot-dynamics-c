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
   (median 9.8x across 39 floating-point measurements) — not because Hazard3 is
   a weak core, but because **the RP2350's RISC-V cores have no FPU**. Every
   `float` operation is emulated in software. An integer-only control
   measurement in the same run shows RISC-V *faster*, which isolates floating
   point as the entire cause.

2. **On the Arm cores the library is comfortably real-time.** A torque control
   tick (`update_kinematics` + `rnea`) costs 280 µs for an 18-DOF Go2 and 41 µs
   for the 9-DOF spine — 3.6 kHz and 24.4 kHz. On the RISC-V cores the same Go2
   tick costs 3.1 ms, i.e. 322 Hz, well under a 1 kHz torque loop.

3. **Four rounds of structural rewrites bought 1.2–2.2x.** On Go2: CRBA 2.16x,
   ABA 1.87x, `update_kinematics` 1.44x, standalone FK 1.65x, RNEA 1.37x,
   spatial acceleration 1.30x. An operational-space tick went from 1.1 kHz to
   1.9 kHz and forward dynamics from 1.0 kHz to 1.8 kHz. Nothing traded accuracy
   for speed: agreement with Pinocchio stayed at machine precision through every
   round. See [Optimisation history](#optimisation-history).

4. **The code is memory-bound, not multiply-bound**, and finding that out
   changed what worked. Two rounds of cutting multiply counts left RNEA exactly
   where it started; one round of cutting *loads* moved it 18%.

5. **ABA scales almost perfectly linearly** — 14.8, 13.7 and 12.9 µs per link on
   the three platforms that have inertial data, across a 4-link chain and a
   31-link branched tree.

6. **Soft-float execution time is data-dependent.** An operation that does
   provably identical work spans 8.29–13.67 µs on RISC-V depending on the
   *values* flowing through it, while staying flat at 1.05–1.06 µs on Arm. That
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
| `update_kinematics` | 16.18 | 23.17 | 55.48 | 155.17 |
| `fk_frame` | 9.71 | 13.08 | 29.18 | 14.81 |
| `jacobian_world` | 3.05 | 8.37 | 7.89 | 9.27 |
| `jacobian_local` | 4.35 | 13.09 | 11.64 | 18.50 |
| `rnea` | 13.25 | 17.80 | 42.35 | 125.03 |
| `aba` | n/a | 59.03 | 136.77 | 399.64 |
| `crba` | 19.32 | 31.90 | 94.55 | 234.43 |
| `gravity` | 9.43 | 12.59 | 29.67 | 86.77 |
| `spatial_accel` | 5.92 | 7.56 | 17.10 | 50.01 |
| `spatial_velocity` | 1.06 | 1.05 | 1.05 | 1.06 |
| *`_heap_probe`* (control) | 6.51 | 7.03 | 7.87 | 11.67 |

## Results — RISC-V (Hazard3 @ 150 MHz)

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 125.77 | 224.22 | 529.11 | 1458.83 |
| `fk_frame` | 87.18 | 154.00 | 336.30 | 170.69 |
| `jacobian_world` | 12.61 | 60.40 | 50.49 | 60.83 |
| `jacobian_local` | 29.11 | 163.13 | 138.47 | 208.79 |
| `rnea` | 95.84 | 225.64 | 556.07 | 1650.28 |
| `aba` | n/a | 668.79 | 1676.84 | 4239.09 |
| `crba` | 137.60 | 323.22 | 1235.55 | 2293.00 |
| `gravity` | 52.96 | 96.86 | 261.39 | 722.09 |
| `spatial_accel` | 40.39 | 76.70 | 167.66 | 478.53 |
| `spatial_velocity` | 8.29 | 13.65 | 13.67 | 13.63 |
| *`_heap_probe`* (control) | 6.05 | 6.45 | 6.89 | 9.81 |

## RISC-V ÷ Arm

Higher means RISC-V is slower.

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 7.77x | 9.68x | 9.54x | 9.40x |
| `fk_frame` | 8.98x | 11.77x | 11.53x | 11.53x |
| `jacobian_world` | 4.13x | 7.22x | 6.40x | 6.56x |
| `jacobian_local` | 6.69x | 12.46x | 11.90x | 11.29x |
| `rnea` | 7.23x | 12.68x | 13.13x | 13.20x |
| `aba` | -- | 11.33x | 12.26x | 10.61x |
| `crba` | 7.12x | 10.13x | 13.07x | 9.78x |
| `gravity` | 5.62x | 7.69x | 8.81x | 8.32x |
| `spatial_accel` | 6.82x | 10.14x | 9.80x | 9.57x |
| `spatial_velocity` | 7.82x | 13.00x | 13.02x | 12.86x |
| **`_heap_probe`** (integer only) | **0.93x** | **0.92x** | **0.88x** | **0.84x** |

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
| `spine` | 9 | 17.80 µs | 59.03 µs | 3.32x | 31.90 µs | 1.85x | 14.8 µs |
| `xarm7` | 7 | 42.35 µs | 136.77 µs | 3.23x | 94.55 µs | 1.45x | 13.7 µs |
| `go2` | 18 | 125.03 µs | 399.64 µs | 3.20x | 234.43 µs | 1.70x | 12.9 µs |

`algo_transform_inertia_accumulate` runs once per link and evaluates `XᵀIX`.
Rewriting it against the block structure (see
[Optimisation history](#optimisation-history)) took CRBA down by half, but it is
still the single largest cost in both algorithms.

ABA's per-link cost is nearly flat across a 4-link serial chain, a 10-link arm
and a 31-link branched tree — 14.8, 13.7, 12.9 µs — which confirms that this one
function, not the tree walking around it, is where the time goes.

**On ABA vs CRBA.** ABA being ~2x CRBA is *not* by itself evidence of a problem,
and an earlier version of this document overstated that. The fair comparison for
forward dynamics is ABA against CRBA **plus** the nonlinear terms **plus** an
nv x nv factorisation: for Go2 that is 234.43 + ~125 = 359 µs before the solve,
against ABA's 400 µs. The two routes are close at this size, which is roughly
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
| `simple_arm` | 3 | 3 | 9.71 µs | 3.24 |
| `spine` | 4 | 4 | 13.08 µs | 3.27 |
| `go2` | 31 | 5 | 14.81 µs | 2.96 |
| `xarm7` | 10 | 10 | 29.18 µs | 2.92 |

If you only need one frame's pose, this is much cheaper than a full
`update_kinematics` — 15 µs against 155 µs on Go2.

### `update_kinematics` is the largest single kinematics item

It is the only algorithm that touches every link unconditionally, and the only
one that calls `sinf`/`cosf` — once per actuated joint. At 155 µs for Go2 it is
now the most expensive item after ABA and CRBA, having overtaken RNEA. It is called once per control tick regardless of
how many algorithms follow, which is the entire point of the design.

### `rd_gravity()` is now genuinely cheaper than RNEA

It runs the same recursion with the cached velocities suppressed, which drops
the Coriolis cross-products and the `I v` bias force. That shows up as a
consistent **0.69–0.71x** of full RNEA across all four platforms. Before v0.2.0
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
**1.06, 1.05, 1.05, 1.06 µs** across the four platforms, flat to within the
timer's resolution.

On RISC-V the same call takes **8.29, 13.65, 13.67, 13.63 µs** — a 1.65x spread
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
| `simple_arm` | 2 | 29.4 µs | 34.0 kHz | 222 µs | 4.5 kHz |
| `spine` | 9 | 41.0 µs | 24.4 kHz | 450 µs | 2.2 kHz |
| `xarm7` | 7 | 97.8 µs | 10.2 kHz | 1085 µs | 922 Hz |
| `go2` | 18 | 280.2 µs | 3.6 kHz | 3109 µs | 322 Hz |

Adding CRBA, for operational-space or inverse-dynamics control that needs the
mass matrix, on Arm: spine 72.9 µs (13.7 kHz), xarm7 192.4 µs (5.2 kHz), Go2
514.6 µs (1.9 kHz).

A **simulation** tick instead — `update_kinematics` + `aba`:

| Robot | dof | Arm | max rate |
|---|---|---|---|
| `spine` | 9 | 82.2 µs | 12.2 kHz |
| `xarm7` | 7 | 192.2 µs | 5.2 kHz |
| `go2` | 18 | 554.8 µs | 1.8 kHz |

Go2 forward dynamics runs at 1.8 kHz on one Arm core, up from 1.0 kHz before the
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
| `rnea` | 0.03 | 0.04 | 0.11 | 0.32 |
| `aba` | n/a | 0.23 | 0.50 | 1.23 |
| `crba` | 0.05 | 0.09 | 0.28 | 0.70 |

Roughly 265–465x the Arm core's throughput, about what the clock ratio and a
wide out-of-order pipeline predict. Useful mostly as a sanity check that the
algorithms are not pathologically slow.

---

## Optimisation history

Four rewrites, each measured on the board in isolation so the attribution is
clean, and each validated against Pinocchio before and after. Agreement stayed
at machine precision throughout, so nothing here trades accuracy for speed.

The order matters, because the first two rounds were aimed at the wrong thing.
Counting the instruction mix in the linked image after round 2 showed **1650
`vldr` and 614 `vstr` against 1695 arithmetic instructions** — more memory
traffic than arithmetic. FMA was already being emitted (1277 `vfma`; GCC
defaults `-ffp-contract=fast` under `gnu11`), so there was nothing to win there
either. This code is memory-bound, and rounds 3 and 4 targeted loads instead of
multiplies.

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

### Round 3 — fewer loads, not fewer multiplies

Two changes, both aimed at memory traffic.

A rigid body's spatial inertia is determined by **ten** numbers, not thirty-six:

```
I = [[ m*1,    -m[c]x ]        I*v = [ m(v_lin - c x w)      ]
     [ m[c]x,   J     ]]              [ m(c x v_lin) + J w    ]
```

`rd_mat6_vec` was loading all 36 entries for every `I·v` product. The packed form
costs 27 multiplies and 10 loads. RNEA does this twice per link, and it is the
one algorithm neither earlier round had touched.

The congruence also got its temporaries cut from twelve 3x3 arrays to three.
Round 1 had reduced its multiply count but paid for it in register spills — more
than 32 single-precision registers of live values on a core that has exactly 32.

And `rd_aba` stopped copying a 36-float articulated inertia per link before
modifying it; the original is dead immediately afterwards.

### Round 4 — stop recomputing what the cache already holds

`algo_joint_velocity()` rebuilt `v_i - Ad(Ti) v_parent` — one spatial transform
and six subtractions — and RNEA, ABA and `rd_spatial_acceleration` each call it
once per link. But `rd_update_kinematics` had already formed exactly that
quantity on its way to `v_i`. Caching the scalar joint velocity turns the
recomputation into `S[k] * vj`: six multiplies, no transform, no reload of `Ti`.
Costs one float per link of state.

### Measured, Arm Cortex-M33, Go2

Microseconds per call. "v0.2.0" is before any of this.

| Algorithm | v0.2.0 | round 1 | round 2 | round 3 | round 4 | total |
|---|---|---|---|---|---|---|
| `crba` | 505.24 | 246.41 | 246.69 | 234.30 | 234.43 | **2.16x** |
| `aba` | 746.57 | 488.62 | 487.21 | 416.00 | 399.64 | **1.87x** |
| `update_kinematics` | 224.14 | 223.04 | 153.53 | 154.44 | 155.17 | **1.44x** |
| `fk_frame` | 24.37 | 24.51 | 14.65 | 14.67 | 14.81 | **1.65x** |
| `rnea` | 171.47 | 172.09 | 172.05 | 141.02 | 125.03 | **1.37x** |
| `gravity` | 106.54 | 106.91 | 107.23 | 86.20 | 86.77 | **1.23x** |
| `spatial_accel` | 64.88 | 64.90 | 65.52 | 65.66 | 50.01 | **1.30x** |
| `jacobian_world` | 9.47 | 9.15 | 9.23 | 9.27 | 9.27 | 1.02x |

Each round moves only what it targets: round 1 the two algorithms that use the
congruence, round 2 the two that build transforms, round 3 the two that multiply
by an inertia, round 4 the three that wanted the joint velocity. Everything else
stays within 1%, which is about the harness's noise floor.

Note RNEA sitting at exactly 1.00x after two rounds of multiply-count work, then
moving 27% once the target became loads. That is the whole lesson of this
exercise in one row.

### What it buys a control loop, Go2 on one Arm core

| Tick | before | after | |
|---|---|---|---|
| torque, `update_kinematics` + `rnea` | 395.6 µs — 2.5 kHz | 280.2 µs — 3.6 kHz | **1.41x** |
| operational space, + `crba` | 900.9 µs — 1.1 kHz | 514.6 µs — 1.9 kHz | **1.75x** |
| forward dynamics, `update_kinematics` + `aba` | 970.7 µs — 1.0 kHz | 554.8 µs — 1.8 kHz | **1.75x** |

The RISC-V cores gain similarly — Go2 CRBA 1.71x, RNEA 1.24x, ABA 1.48x — and
their integer control tightened from 0.73x to 0.84x of the Arm time, because the
float work shrank while the allocator probe did not.

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

The easy structural wins are spent. What remains is smaller and each item is
worth a few percent, so the honest summary is that this is the knee of the curve
for the current design:

1. **Store the composite/articulated inertia symmetric.** 21 floats instead of
   36 would cut the congruence's memory traffic by 40%, which given the
   instruction mix is where the remaining CRBA and ABA time is. It is the one
   item likely worth more than 10%, and it is also the most invasive, since
   every accessor changes.

2. **Exploit that `S` has exactly one non-zero entry.** Joint axes are restricted
   to ±X/±Y/±Z, so `S` is a unit basis vector: `rd_mat6_vec(I, S, U)` is a column
   extract, and `S · f` is a single element. Worth roughly 3–5% on CRBA and ABA,
   and it needs the non-zero index cached in the state.

3. **Fold the joint rotation into the offset compose.** A rotation about a
   coordinate axis mixes only two columns, so `T_joint_offset * T_motion` is 12
   multiplies rather than 36, and `T_motion` never needs to exist. Only helps the
   actuated links — 12 of Go2's 31 — so ~3% of `update_kinematics`.

4. **Cache `sincos` per joint.** `rd_fk_frame` recomputes what
   `update_kinematics` already evaluated.

5. **Run dynamics on the Arm cores.** Nothing in the library needs changing —
   this is a deployment decision, and it is worth 4–13x.

6. **If the RISC-V cores are unavoidable, consider fixed point.** With no FPU, a
   Q-format implementation would very likely beat soft float, and would restore
   deterministic timing.

Already done: scratch moved into `rd_state_t` and CRBA's duplicate parent-chain
walk removed (v0.2.0); the inertia congruence, the SE(3) compose and the
coordinate-axis rotation specialised (v0.3.0); the packed spatial inertia,
reduced congruence temporaries, ABA's in-place downdate and the cached joint
velocity (v0.4.0).

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
