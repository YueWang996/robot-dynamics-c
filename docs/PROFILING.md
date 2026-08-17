# Profiling RobotDynamics on the RP2350

Measured performance of every algorithm in the library, on both CPU
architectures of a Raspberry Pi Pico 2, across four robot platforms.

All raw data is in [`benchmark/results/`](../benchmark/results/); every table
below is generated from it by [`tools/report.py`](../tools/report.py). For the
correctness side, see [VALIDATION.md](VALIDATION.md) — every algorithm measured
here agrees with Pinocchio to machine precision.

---

## Summary

1. **The Arm cores are 4–14x faster than the RISC-V cores** on this workload
   (median 10.3x across 36 floating-point measurements) — not because Hazard3 is
   a weak core, but because **the RP2350's RISC-V cores have no FPU**. Every
   `float` operation is emulated in software. An integer-only control
   measurement in the same run shows RISC-V up to 1.5x *faster*, which isolates
   floating point as the entire cause.

2. **On the Arm cores the library is comfortably real-time.** A full torque
   control tick (`update_kinematics` + `rnea`) costs 395 µs for an 18-DOF Go2
   and 58 µs for the 9-DOF spine — 2.5 kHz and 17.2 kHz. On the RISC-V cores the
   same Go2 tick costs 4.1 ms, i.e. 246 Hz, well under a 1 kHz torque loop.

3. **CRBA is the most expensive algorithm by a wide margin** — 1.9x the cost of
   RNEA on the smallest robot, rising to 3.0x on Go2. It is the clearest
   optimisation target.

4. **Soft-float execution time is data-dependent.** An operation that does
   provably identical work spans 8.73–13.58 µs on RISC-V depending on the
   *values* flowing through it, while staying flat at 0.99–1.00 µs on Arm. That
   matters if you need a worst-case execution time bound.

5. **Both `rd_rnea_cached` and `rd_crba_cached` call `malloc`/`free` on every
   invocation.** On Arm this is 5.6–22.2% of RNEA's total cost, and it makes the
   library's own `RD_USE_STATIC_ALLOC=1` mode non-functional for those two
   algorithms.

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

The floating-base platforms are driven through `rd_update_kinematics_fb` with a
non-identity base pose and a non-zero base twist, and their RNEA and spatial
acceleration calls receive a non-zero base acceleration. These are the costs of
the real floating-base path, not of a base pinned at the origin.

`aba` (forward dynamics) is in bard but **not implemented in this library**, so
it has no row here.

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
* The cached algorithms follow the library's intended pattern: one
  `rd_update_kinematics_fb` primes `rd_state_t`, then each algorithm reuses it.
  That priming call is timed separately as its own line item.

**Repeatability.** The Arm suite was flashed and run twice from scratch. Across
all 44 measurements the largest deviation between runs was **below 0.005%**;
most were bit-identical. The workload is deterministic and the board has no
frequency scaling, so run-to-run noise is not a factor in anything below.

---

## Results — Arm (Cortex-M33 @ 150 MHz)

Microseconds per call. Multiply by 150 for cycles.

| Algorithm | simple_arm<br><sub>3L / 2 dof</sub> | spine<br><sub>4L / 9 dof</sub> | xarm7<br><sub>10L / 7 dof</sub> | go2<br><sub>31L / 18 dof</sub> |
|---|---|---|---|---|
| `update_kinematics` | 21.89 | 31.27 | 75.72 | 224.42 |
| `fk_frame` | 15.72 | 21.57 | 49.75 | 24.55 |
| `jacobian_world` | 2.91 | 8.25 | 7.75 | 9.15 |
| `jacobian_local` | 4.23 | 13.00 | 11.52 | 18.40 |
| `rnea` | 20.53 | 26.79 | 58.65 | 170.27 |
| `gravity_comp` | 20.29 | 26.35 | 57.74 | 167.27 |
| `crba` | 38.82 | 64.00 | 175.51 | 517.19 |
| `spatial_accel` | 8.49 | 10.47 | 22.01 | 62.71 |
| `spatial_velocity` | 0.99 | 1.00 | 1.00 | 0.99 |
| *`_heap_rnea`* (probe) | 4.57 | 5.06 | 5.79 | 9.62 |
| *`_heap_crba`* (probe) | 1.49 | 1.46 | 1.46 | 1.49 |

## Results — RISC-V (Hazard3 @ 150 MHz)

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 179.17 | 307.43 | 734.12 | 2015.03 |
| `fk_frame` | 143.03 | 241.64 | 542.34 | 270.72 |
| `jacobian_world` | 12.38 | 60.47 | 49.34 | 60.72 |
| `jacobian_local` | 28.77 | 162.85 | 137.06 | 208.08 |
| `rnea` | 123.64 | 281.31 | 676.08 | 2046.28 |
| `gravity_comp` | 121.78 | 267.16 | 666.51 | 1982.25 |
| `crba` | 239.73 | 554.24 | 1825.47 | 4122.66 |
| `spatial_accel` | 51.38 | 111.41 | 243.46 | 718.94 |
| `spatial_velocity` | 8.73 | 13.54 | 13.57 | 13.58 |
| *`_heap_rnea`* (probe) | 3.83 | 4.22 | 4.45 | 6.33 |
| *`_heap_crba`* (probe) | 1.55 | 1.56 | 1.55 | 1.60 |

## RISC-V ÷ Arm

Higher means RISC-V is slower.

| Algorithm | simple_arm | spine | xarm7 | go2 |
|---|---|---|---|---|
| `update_kinematics` | 8.19x | 9.83x | 9.70x | 8.98x |
| `fk_frame` | 9.10x | 11.20x | 10.90x | 11.03x |
| `jacobian_world` | 4.26x | 7.33x | 6.36x | 6.63x |
| `jacobian_local` | 6.80x | 12.53x | 11.90x | 11.31x |
| `rnea` | 6.02x | 10.50x | 11.53x | 12.02x |
| `gravity_comp` | 6.00x | 10.14x | 11.54x | 11.85x |
| `crba` | 6.18x | 8.66x | 10.40x | 7.97x |
| `spatial_accel` | 6.05x | 10.64x | 11.06x | 11.47x |
| `spatial_velocity` | 8.79x | 13.54x | 13.57x | 13.67x |
| **`_heap_rnea`** (integer only) | **0.84x** | **0.83x** | **0.77x** | **0.66x** |
| **`_heap_crba`** (integer only) | **1.04x** | **1.07x** | **1.06x** | **1.07x** |

The last two rows are the control. They exercise `malloc`/`free` — pointer
arithmetic and free-list walking, no floating point at all — and RISC-V ties or
wins every one of them, running the allocator up to 1.5x faster than the M33.
Hazard3 is not a slow core. It simply has no FPU, and this library is almost
entirely floating-point arithmetic.

---

## Where the time goes

### CRBA dominates

CRBA costs **1.9–3.0x** what RNEA costs, and the ratio grows with model size.
For Go2 on Arm that is 517 µs against RNEA's 170 µs.

The cost sits in `algo_transform_inertia_accumulate`, which is called once per
link and evaluates `XᵀIX` using two **dense** 6x6 `rd_mat6_mul` calls — 432
multiplies and 360 adds per link, before the adjoint construction and the
transpose. A spatial inertia is symmetric and its adjoint is block-structured
with a zero block, so most of that arithmetic multiplies known zeros.

There is a second, smaller cost: the mass-matrix assembly loop propagates each
column's force to the root **twice** for floating-base models — once in the
floating-base coupling block and again in the joint coupling block, walking the
same parent chain both times.

### `fk_frame` scales with depth, not link count

Go2 has three times the links of xarm7 but its standalone FK is **twice as
fast**. `rd_fk_frame` walks only the root-to-frame path, and Go2's tree is four
shallow branches of depth 5 while xarm7 is a single serial chain of depth 10.

Sorted by path length rather than link count, the cost is almost exactly linear
at about 5 µs per level on Arm:

| Platform | Links | Path length | `fk_frame` | µs per level |
|---|---|---|---|---|
| `simple_arm` | 3 | 3 | 15.72 µs | 5.24 |
| `spine` | 4 | 4 | 21.57 µs | 5.39 |
| `go2` | 31 | 5 | 24.55 µs | 4.91 |
| `xarm7` | 10 | 10 | 49.75 µs | 4.97 |

If you only need one frame's pose, this is much cheaper than a full
`update_kinematics` — 25 µs against 224 µs on Go2.

### `update_kinematics` is the single largest line item

It is the only algorithm that touches every link unconditionally, and the only
one that calls `sinf`/`cosf` — once per actuated joint, through
`rd_rot_axis_angle`. At 224 µs for Go2 it costs more than RNEA itself. It is
however called once per control tick regardless of how many algorithms follow,
which is the entire point of the design.

### Allocator traffic inside the control loop

| Robot | RNEA | of which heap | CRBA | of which heap |
|---|---|---|---|---|
| simple_arm | 20.53 µs | 4.57 µs (**22.2%**) | 38.82 µs | 1.49 µs (3.8%) |
| spine | 26.79 µs | 5.06 µs (**18.9%**) | 64.00 µs | 1.46 µs (2.3%) |
| xarm7 | 58.65 µs | 5.79 µs (9.9%) | 175.51 µs | 1.46 µs (0.8%) |
| go2 | 170.27 µs | 9.62 µs (5.6%) | 517.19 µs | 1.49 µs (0.3%) |

(Arm figures.) `rd_rnea_cached` calls `RD_CALLOC` twice and `RD_FREE` twice per
invocation; `rd_crba_cached` calls `RD_MALLOC`/`RD_FREE` once. For small robots
— exactly the ones you would put on a microcontroller — this is up to a fifth of
the call. It is also a real-time hazard independent of its cost: `malloc` may
take a lock and can fragment over a long run.

### Soft-float timing is data-dependent

`rd_get_spatial_velocity_cached` does the same fixed amount of work for every
robot: one memcpy and one 6-vector spatial transform. Its cost confirms that on
Arm — **0.99, 1.00, 1.00, 0.99 µs** across the four platforms, flat to within
the timer's resolution.

On RISC-V the same call takes **8.73, 13.54, 13.57, 13.58 µs** — a 1.56x spread
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
| `simple_arm` | 2 | 42.4 µs | 23.6 kHz | 303 µs | 3.3 kHz |
| `spine` | 9 | 58.1 µs | 17.2 kHz | 589 µs | 1.7 kHz |
| `xarm7` | 7 | 134.4 µs | 7.4 kHz | 1410 µs | 709 Hz |
| `go2` | 18 | 394.7 µs | 2.5 kHz | 4061 µs | 246 Hz |

Adding CRBA, for operational-space or inverse-dynamics control that needs the
mass matrix, on Arm: spine 122 µs (8.2 kHz), xarm7 310 µs (3.2 kHz), Go2 912 µs
(1.1 kHz).

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
| `update_kinematics` | 0.04 | 0.07 | 0.16 | 0.41 |
| `fk_frame` | 0.04 | 0.05 | 0.12 | 0.06 |
| `rnea` | 0.07 | 0.10 | 0.20 | 0.48 |
| `crba` | 0.08 | 0.13 | 0.36 | 0.91 |

Roughly 280–570x the Arm core's throughput, about what the clock ratio and a
wide out-of-order pipeline predict. Useful mostly as a sanity check that the
algorithms are not pathologically slow.

---

## Known issues

Properties of the library as it currently stands, surfaced by building,
measuring and validating it.

### `rd_gravity_compensation()` does not return gravity terms

```c
rd_status_t rd_gravity_compensation(...) { return rd_rnea_cached(chain, state, NULL, NULL, gravity, tau_g); }
rd_status_t rd_nonlinear_terms(...)      { return rd_rnea_cached(chain, state, NULL, NULL, gravity, tau_nle); }
```

The two are character-for-character the same call. Because the cached
`rd_state_t` already has `q̇` baked into `state->v`, both return the full
nonlinear term `C(q,q̇)q̇ + g(q)`, not `g(q)`. To get gravity alone you must
re-run `rd_update_kinematics` with `qd = 0`. The measured `gravity_comp` row is
therefore the same work as `rnea`, which is exactly what the numbers show
(167.27 vs 170.27 µs on Go2).

### `RD_USE_STATIC_ALLOC=1` breaks RNEA and CRBA

Under that flag `RD_MALLOC`/`RD_CALLOC` expand to `NULL`. `rd_rnea_cached` and
`rd_crba_cached` allocate scratch on every call, so both immediately return
`RD_ERR_ALLOC_FAILED`. The library's advertised no-malloc mode does not
currently work for its two main dynamics entry points.

Fixing this and removing the per-call allocation are the same change: move the
scratch buffers into `rd_state_t`, where the rest of the per-tick workspace
already lives.

### Topological ordering relies on an undocumented invariant

`rd_chain_build` builds `topo_order` with a hand-unrolled BFS that only reaches
depth 3. Anything deeper falls through to a fallback that appends the remaining
nodes **in link-index order**. That is a valid topological order only because
every model declares parents before children. A model with a child at a lower
index than its parent would silently produce a wrong ordering rather than an
error. `tools/urdf2c.py` guarantees the invariant and
`tools/test_urdf2c.py::test_parent_precedes_child` asserts it; a hand-written
model could still violate it.

### Fixed during this work

* **CRBA transformed composite inertia in the wrong direction** — mass matrix
  was 13–40% wrong. See [VALIDATION.md](VALIDATION.md).
* **`rd_update_kinematics` ignored the floating base**, forcing the root to the
  identity pose with zero velocity. Now `rd_update_kinematics_fb` takes
  `q_base`/`qd_base`, and RNEA no longer double-rotates gravity.
* **`tools/urdf2c.py` emitted joints breadth-first**, which scrambled `q`
  against Pinocchio and bard on branched robots.
* **`RD_USE_SINGLE_PRECISION=0` did not give double precision** — `rd_math.h`
  hardcoded the `float` libm variants.
* **The repository did not compile**: `robot_dynamics.h` never included
  `rd_algorithms.h`, `test_main.c` targeted a removed API, and the top-level
  `install()` list named four headers that do not exist.

---

## Ranked optimisation opportunities

1. **Move RNEA/CRBA scratch into `rd_state_t`.** Removes 5.6–22.2% from RNEA,
   makes the control loop allocation-free, and fixes `RD_USE_STATIC_ALLOC`.
   Small, contained change with the best cost-to-benefit ratio here.

2. **Exploit structure in the CRBA inertia transform.** `XᵀIX` with a symmetric
   `I` and a block-structured adjoint needs roughly a quarter of the dense-6x6
   arithmetic currently spent. CRBA is the most expensive algorithm, so this is
   the largest absolute win.

3. **Don't propagate each CRBA column to the root twice.** The floating-base and
   joint coupling loops duplicate the same parent-chain walk.

4. **Cache `sincos` per joint in `rd_state_t`.** `update_kinematics` is the
   largest single line item and calls `sinf`/`cosf` once per joint;
   `rd_fk_frame` then recomputes the same values.

5. **Run dynamics on the Arm cores.** Nothing in the library needs changing —
   this is a deployment decision, and it is worth 4–14x.

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
