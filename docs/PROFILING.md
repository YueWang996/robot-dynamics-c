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
   (median 9.5x across 39 floating-point measurements) — not because Hazard3 is
   a weak core, but because **the RP2350's RISC-V cores have no FPU**. Every
   `float` operation is emulated in software. An integer-only control
   measurement in the same run shows RISC-V up to 1.4x *faster*, which isolates
   floating point as the entire cause.

2. **On the Arm cores the library is comfortably real-time.** A torque control
   tick (`update_kinematics` + `rnea`) costs 396 µs for an 18-DOF Go2 and 54 µs
   for the 9-DOF spine — 2.5 kHz and 18.4 kHz. On the RISC-V cores the same Go2
   tick costs 4.1 ms, i.e. 246 Hz, well under a 1 kHz torque loop.

3. **Forward dynamics costs more than the mass matrix.** `rd_aba` runs at
   4.1–4.4x `rd_rnea` and **1.4–1.6x `rd_crba`**, which is backwards from what
   ABA is supposed to buy you. Both algorithms spend most of their time in the
   same dense 6x6 inertia congruence, and CRBA is the one that has been thought
   about. This is the clearest optimisation target in the library.

4. **ABA scales almost perfectly linearly** — 24.0, 24.2 and 24.1 µs per link on
   the three platforms that have inertial data, across a 4-link chain and a
   31-link branched tree.

5. **Soft-float execution time is data-dependent.** An operation that does
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
| `update_kinematics` | 21.98 | 30.91 | 76.11 | 224.14 |
| `fk_frame` | 15.60 | 21.45 | 49.47 | 24.37 |
| `jacobian_world` | 3.06 | 8.57 | 8.00 | 9.47 |
| `jacobian_local` | 4.38 | 13.32 | 11.79 | 18.71 |
| `rnea` | 17.33 | 23.44 | 56.99 | 171.47 |
| `aba` | n/a | 95.83 | 241.62 | 746.57 |
| `crba` | 37.45 | 59.16 | 176.19 | 505.24 |
| `gravity` | 11.39 | 15.23 | 36.13 | 106.54 |
| `spatial_accel` | 6.96 | 9.21 | 21.55 | 64.88 |
| `spatial_velocity` | 1.05 | 1.06 | 1.06 | 1.05 |
| *`_heap_probe`* (control) | 6.51 | 7.03 | 7.87 | 11.70 |

## Results — RISC-V (Hazard3 @ 150 MHz)

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 179.01 | 302.56 | 724.08 | 2021.88 |
| `fk_frame` | 143.09 | 236.50 | 531.98 | 265.55 |
| `jacobian_world` | 12.64 | 60.38 | 50.47 | 61.21 |
| `jacobian_local` | 29.15 | 163.11 | 138.45 | 209.16 |
| `rnea` | 120.46 | 271.95 | 674.90 | 2041.12 |
| `aba` | n/a | 906.27 | 2380.12 | 6292.78 |
| `crba` | 237.48 | 504.82 | 1818.91 | 3921.03 |
| `gravity` | 60.99 | 105.25 | 282.49 | 793.11 |
| `spatial_accel` | 49.70 | 105.91 | 244.64 | 726.21 |
| `spatial_velocity` | 8.29 | 13.65 | 13.67 | 13.63 |
| *`_heap_probe`* (control) | 5.92 | 6.34 | 6.67 | 8.55 |

## RISC-V ÷ Arm

Higher means RISC-V is slower.

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 8.14x | 9.79x | 9.51x | 9.02x |
| `fk_frame` | 9.17x | 11.02x | 10.75x | 10.90x |
| `jacobian_world` | 4.13x | 7.04x | 6.31x | 6.46x |
| `jacobian_local` | 6.65x | 12.25x | 11.74x | 11.18x |
| `rnea` | 6.95x | 11.60x | 11.84x | 11.90x |
| `aba` | -- | 9.46x | 9.85x | 8.43x |
| `crba` | 6.34x | 8.53x | 10.32x | 7.76x |
| `gravity` | 5.36x | 6.91x | 7.82x | 7.44x |
| `spatial_accel` | 7.14x | 11.50x | 11.35x | 11.19x |
| `spatial_velocity` | 7.87x | 12.88x | 12.89x | 12.94x |
| **`_heap_probe`** (integer only) | **0.91x** | **0.90x** | **0.85x** | **0.73x** |

The last row is the control. It exercises `malloc`/`free` — pointer arithmetic
and free-list walking, no floating point at all — and RISC-V wins every one of
them, running the allocator up to 1.4x faster than the M33. Hazard3 is not a
slow core. It simply has no FPU, and this library is almost entirely
floating-point arithmetic. (The library itself no longer allocates anywhere;
this probe exists only as an architecture comparison that touches no floats.)

---

## Where the time goes

### The 6x6 inertia congruence dominates both CRBA and ABA

| Robot | dof | RNEA | ABA | ABA/RNEA | CRBA | ABA/CRBA | ABA per link |
|---|---|---|---|---|---|---|---|
| `spine` | 9 | 23.44 µs | 95.83 µs | 4.09x | 59.16 µs | 1.62x | 24.0 µs |
| `xarm7` | 7 | 56.99 µs | 241.62 µs | 4.24x | 176.19 µs | 1.37x | 24.2 µs |
| `go2` | 18 | 171.47 µs | 746.57 µs | 4.35x | 505.24 µs | 1.48x | 24.1 µs |

Forward dynamics costing **more** than assembling the mass matrix is the wrong
way round — ABA exists precisely to avoid building and factoring `M`. Both
algorithms are dominated by the same function.

`algo_transform_inertia_accumulate` runs once per link and evaluates `XᵀIX` with
two **dense** 6x6 `rd_mat6_mul` calls: 432 multiplies and 360 adds per link,
before the adjoint construction and the transpose. A spatial inertia is
symmetric and its adjoint is block-structured with a zero block, so most of that
arithmetic multiplies known zeros. Exploiting that structure would cut roughly
three quarters of the work out of the two most expensive algorithms at once.

The per-link cost being identical to three significant figures across a 4-link
serial chain and a 31-link branched tree confirms that this single function, not
the tree walking around it, is where ABA's time goes.

### `fk_frame` scales with depth, not link count

Go2 has three times the links of xarm7 but its standalone FK is **twice as
fast**. `rd_fk_frame` walks only the root-to-frame path, and Go2's tree is four
shallow branches of depth 5 while xarm7 is a single serial chain of depth 10.

Sorted by path length rather than link count, the cost is almost exactly linear
at about 5 µs per level on Arm:

| Platform | Links | Path length | `fk_frame` | µs per level |
|---|---|---|---|---|
| `simple_arm` | 3 | 3 | 15.60 µs | 5.20 |
| `spine` | 4 | 4 | 21.45 µs | 5.36 |
| `go2` | 31 | 5 | 24.37 µs | 4.87 |
| `xarm7` | 10 | 10 | 49.47 µs | 4.95 |

If you only need one frame's pose, this is much cheaper than a full
`update_kinematics` — 24 µs against 224 µs on Go2.

### `update_kinematics` is the largest single kinematics item

It is the only algorithm that touches every link unconditionally, and the only
one that calls `sinf`/`cosf` — once per actuated joint, through
`rd_rot_axis_angle`. At 224 µs for Go2 it costs more than RNEA itself. It is
however called once per control tick regardless of how many algorithms follow,
which is the entire point of the design.

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
**1.05, 1.06, 1.06, 1.05 µs** across the four platforms, flat to within the
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
| `simple_arm` | 2 | 39.3 µs | 25.4 kHz | 299 µs | 3.3 kHz |
| `spine` | 9 | 54.4 µs | 18.4 kHz | 575 µs | 1.7 kHz |
| `xarm7` | 7 | 133.1 µs | 7.5 kHz | 1399 µs | 715 Hz |
| `go2` | 18 | 395.6 µs | 2.5 kHz | 4063 µs | 246 Hz |

Adding CRBA, for operational-space or inverse-dynamics control that needs the
mass matrix, on Arm: spine 114 µs (8.8 kHz), xarm7 309 µs (3.2 kHz), Go2 901 µs
(1.1 kHz).

A **simulation** tick instead — `update_kinematics` + `aba` — is the expensive
one, because ABA is:

| Robot | dof | Arm | max rate | RISC-V | max rate |
|---|---|---|---|---|---|
| `spine` | 9 | 126.7 µs | 7.9 kHz | 1209 µs | 827 Hz |
| `xarm7` | 7 | 317.7 µs | 3.1 kHz | 3104 µs | 322 Hz |
| `go2` | 18 | 970.7 µs | 1.0 kHz | 8315 µs | 120 Hz |

Go2 forward dynamics lands at 1.03 kHz on an Arm core — it clears a 1 kHz
simulation loop, but only just, and that is the number the congruence
optimisation above would move most.

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
| `update_kinematics` | 0.04 | 0.06 | 0.15 | 0.40 |
| `fk_frame` | 0.04 | 0.05 | 0.12 | 0.06 |
| `rnea` | 0.04 | 0.06 | 0.15 | 0.44 |
| `aba` | n/a | 0.20 | 0.50 | 1.49 |
| `crba` | 0.06 | 0.10 | 0.35 | 0.86 |

Roughly 360–620x the Arm core's throughput, about what the clock ratio and a
wide out-of-order pipeline predict. Useful mostly as a sanity check that the
algorithms are not pathologically slow.

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

1. ~~Move RNEA/CRBA scratch into `rd_state_t`.~~ **Done in v0.2.0** — this was
   worth 5.6–22.2% of RNEA and made the control loop allocation-free.

2. **Exploit structure in the inertia congruence.** `XᵀIX` with a symmetric `I`
   and a block-structured adjoint needs roughly a quarter of the dense-6x6
   arithmetic currently spent. It is the dominant cost in *both* CRBA and ABA,
   so this one change is the largest available win by a wide margin — and it is
   what currently makes forward dynamics more expensive than the mass matrix.

3. ~~Don't propagate each CRBA column to the root twice.~~ **Done in v0.2.0** —
   the two parent-chain walks are now one.

4. **Cache `sincos` per joint in `rd_state_t`.** `update_kinematics` is the
   largest single line item and calls `sinf`/`cosf` once per joint;
   `rd_fk_frame` then recomputes the same values.

5. **Run dynamics on the Arm cores.** Nothing in the library needs changing —
   this is a deployment decision, and it is worth 4–13x.

6. **If the RISC-V cores are unavoidable, consider fixed point.** With no FPU, a
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
