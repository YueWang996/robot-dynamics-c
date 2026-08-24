<div align="center">

# RobotDynamics

**Rigid-body dynamics that fits on a microcontroller.**

C99 · `libm` and nothing else · a 1 kHz torque loop for an 18-DOF quadruped on a $5 board

[![Single header](https://img.shields.io/badge/download-single%20header-brightgreen.svg)](../../releases/latest)
![C99](https://img.shields.io/badge/C-99-blue.svg)
[![Licence](https://img.shields.io/badge/licence-Apache%202.0-green.svg)](LICENSE)
[![Targets](https://img.shields.io/badge/targets-Cortex--M4F%20|%20M33%20|%20RISC--V-orange.svg)](#speed)
![Validated](https://img.shields.io/badge/validated%20vs-Pinocchio-purple.svg)

</div>

---

## What it is

A complete rigid-body dynamics stack for robots that carry their own computer.
Hand it a URDF-derived model and one buffer, and it returns torques, Jacobians
and mass matrices fast enough to close the loop on the robot itself.

| | |
|---|---|
| **Algorithms** | Forward kinematics · geometric Jacobian · RNEA · ABA · CRBA · gravity, Coriolis and nonlinear terms · spatial velocity and acceleration |
| **Joints** | Revolute, prismatic, fixed, and a floating base |
| **Frames** | World and body, wherever a reference frame applies |
| **Contacts** | External spatial forces on any link, applied inside the O(n) recursion — `rd_rnea_ext`, `rd_aba_ext` |
| **Actuators** | Reflected rotor inertia per joint, which any geared drive needs — `rd_chain_set_armature` |
| **Linear algebra** | The Cholesky the forward dynamics uses, exposed for task-space work — `rd_cholesky_factor` / `_solve` |
| **Accuracy** | Validated against Pinocchio at **5.5e-15** (float64) and **1.9e-06** (float32) |
| **Precision** | `float` by default, `double` with one flag |
| **Allocation** | None in the control loop. One caller-provided buffer holds every algorithm's scratch |
| **Dependencies** | `libm`. That is the whole list |
| **Footprint** | **40.2 KB** of Cortex-M4 code for the whole library, about 25 KB once `--gc-sections` drops what you do not call |
| **Distribution** | One header. Download it from [Releases](../../releases/latest) and copy it in |
| **Model input** | URDF, through `tools/urdf2c.py` |
| **Interop** | `q`, `qd`, `qdd` and `tau` use Pinocchio's layout, so vectors pass straight to [bard](https://github.com/YueWang996/bard-pytorch-dynamics) and back |

A control tick usually wants several quantities at once, and each of them
begins by walking the kinematic tree. `rd_update_kinematics` walks it once and
every algorithm afterwards reads that cache, which is where most of the speed
comes from.

> [!NOTE]
> Pre-1.0, with three gaps worth stating. Prismatic joints are implemented and
> unvalidated, since the reference robots are revolute-only. Joint limits,
> damping and friction are parsed and stored, and no algorithm reads them yet.
> `rd_chain_build()` allocates once at startup, so `RD_STATIC_ALLOC=ON` is not
> supported; the control loop is allocation-free either way.
>
> There is no controller here and no contact solver: this library gives you
> M, h, J, J̇q̇ and a factorisation, and what you build on them is yours.

---

## Using it

Download **`robot_dynamics.h`** from
[Releases](../../releases/latest) and drop it into your project. One file, no
build system, no submodule. In exactly one `.c` file:

```c
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

and include it plainly everywhere else.

> [!TIP]
> **A part with a math accelerator can use it.** `RD_MATH_BACKEND` names a
> header of *yours* that defines `RD_SINCOS` and/or `RD_SQRT`, and it works the
> same in the single-file build as in the tree. Nothing of the sort is bundled
> — the download stays one file with no vendor headers behind it — but
> [`examples/backends/`](examples/backends/) has a worked one for the STM32G4
> CORDIC to read.

```c
#include "robot_dynamics.h"

rd_chain_t chain;
rd_chain_build(&my_model, &chain);          /* once, at startup */

static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));

for (;;) {
    read_encoders(q_joints, qd);
    read_base_estimate(q_base);             /* NULL for a fixed base */

    rd_update_kinematics(&chain, &state, q_base, q_joints, qd);

    rd_rnea(&chain, &state, qdd, NULL, tau);            /* inverse dynamics */
    rd_jacobian(&chain, &state, eef, RD_FRAME_WORLD, J);

    write_torques(tau);
}
```

**Working from the source tree** instead — for CMake projects, or to build the
single header yourself:

```bash
git clone <this repo> && cd RobotDynamics
cmake -B build && cmake --build build && ./build/rd_test   # host smoke test
python3 tools/amalgamate.py --verify                       # dist/robot_dynamics.h
```

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

`amalgamate.py --verify` compiles the header it just wrote three ways and
compares its Cortex-M4 code against the multi-file build, function by function.
The released header is the same library: all 22 public functions come out
instruction for instruction identical.

**Bring in a robot:**

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h --floating-base
```

The converter enforces what the C model requires — axis-aligned joint axes,
15-character link names, parents before children — and fails loudly on anything
it cannot represent.

**Contacts and geared joints.** `rd_rnea_ext` and `rd_aba_ext` take one spatial
force per *link*, in that link's own body frame, and fold it into the recursion
rather than costing a Jacobian per contact. Placing one on a foot works even
though feet are usually fixed links that the chain folded away. `armature` is
the reflected rotor inertia, `n²·I_rotor`, which on a servo or any high-ratio
drive is often larger than the link it turns; URDF has no field for it, so it
starts at zero and `rd_chain_set_armature()` fills it in. Both cost nothing
when unused.

**Conventions worth reading once.** Reversed, these produce a wrong robot
quietly:

| | |
|---|---|
| Base quaternion | `q_base = [x y z qw qx qy qz]`, **scalar first** (Pinocchio and ROS are scalar-last) |
| Base twist and acceleration | The root link's **body frame** |
| Spatial vectors | `[linear, angular]` |
| `qd`, `qdd`, `tau` | Length `nv`, packed, base in the first six |
| `M` | `nv × nv`, fully filled |
| Joint order | Depth-first in URDF declaration order, matching Pinocchio and bard |

**Options:**

| | Default | |
|---|---|---|
| `RD_SINGLE_PRECISION` | `ON` | `float` for `rd_real_t`; `OFF` gives `double` |
| `RD_FAST_TRIG` | on | Polynomial `sin`/`cos`, 169 cycles a pair against libm's 521 at the same float32 accuracy — worth 45% of `update_kinematics` on Go2 |
| `RD_ENABLE_ABA` | `ON` | The articulated-body algorithm. It is the only algorithm with per-node state of its own, so leaving it out takes a link from 70 floats of workspace to 45 and saves 13 KB of flash. `rd_forward_dynamics(RD_FD_CRBA)` still works |
| `RD_MATH_BACKEND` | unset | A header of yours defining `RD_SINCOS` and/or `RD_SQRT`, so a part with a math accelerator can use it. Bring your own; [`examples/backends/`](examples/backends/) has one for the STM32G4 CORDIC to work from |
| `RD_CMSIS_DSP` | `OFF` | CMSIS-DSP `sqrt`; needs `RD_CMSIS_DSP_INCLUDE_DIR` |
| `RD_MAX_LINKS` / `RD_MAX_JOINTS` | 16 / 12 | Bound the static model storage. Go2 needs 31 links |
| `RD_ENABLE_DEBUG` | `OFF` | Assertions and log output |

`./build/rd_test` is a smoke test that needs nothing installed;
`python3 tools/validate.py --urdf-root /path/to/bard` runs the full Pinocchio
cross-check.

> [!TIP]
> **Let an agent drive it.** This repository ships an agent skill at
> [`.claude/skills/robot-dynamics/`](.claude/skills/robot-dynamics/) covering the
> API, the conventions above, and how to verify a change against Pinocchio.
> Claude Code loads it automatically inside this repository. To use it from
> another project, copy that folder into `~/.claude/skills/` and ask for
> RobotDynamics by name.

---

## Speed

Go2 — 18 DOF, 31 links, floating base — microseconds per call, single
precision, `-O3`:

| | **M4F @ 170**<br><sub>STM32G474</sub> | **M4F @ 80**<br><sub>STM32L413</sub> | **M33 @ 150**<br><sub>RP2350</sub> | **Hazard3 @ 150**<br><sub>RP2350</sub> | **RV32 @ 160**<br><sub>ESP32-C6</sub> |
|---|---|---|---|---|---|
| | FPU | FPU | FPU | *no FPU* | *no FPU* |
| `update_kinematics` | 26.9 | 55.5 | 24.9 | 265.3 | 354.7 |
| `fk_frame` | 10.7 | 22.2 | 9.9 | 107.0 | 142.1 |
| `jacobian_world` | 12.2 | 25.4 | 10.3 | 63.8 | 81.0 |
| `jacobian_local` | 19.9 | 41.4 | 16.7 | 177.6 | 224.2 |
| `rnea` | 44.1 | 93.3 | 45.4 | 767.9 | 938.4 |
| `crba` | 54.7 | 113.4 | 55.9 | 530.4 | 678.9 |
| `aba` | 157.5 | 326.9 | 120.2 | 1756.8 | 2211.9 |
| `gravity` | 33.1 | 67.9 | 32.7 | 314.3 | 471.5 |
| `spatial_acceleration` | 19.4 | 40.6 | 17.9 | 202.8 | 275.6 |
| `spatial_velocity` | 4.7 | 9.7 | 4.6 | 52.3 | 72.5 |
| | | | | | |
| **torque tick** | 71 µs<br>14.1 kHz | 149 µs<br>6.7 kHz | 70 µs<br>14.2 kHz | 1033 µs<br>968 Hz | 1293 µs<br>773 Hz |
| **operational space** | 126 µs<br>8.0 kHz | 262 µs<br>3.8 kHz | 126 µs<br>7.9 kHz | 1564 µs<br>640 Hz | 1972 µs<br>507 Hz |
| **forward dynamics** | 184 µs<br>5.4 kHz | 382 µs<br>2.6 kHz | 145 µs<br>6.9 kHz | 2022 µs<br>495 Hz | 2567 µs<br>390 Hz |

Torque tick is `update_kinematics` + `rnea`; operational space adds `crba`;
forward dynamics is `update_kinematics` + the faster of ABA and CRBA for that
robot.

How it scales with the robot, on the STM32G474 at 170 MHz:

| Robot | nv | Torque tick | Operational space | Forward dynamics |
|---|---|---|---|---|
| `spine` | 9, floating | 26 µs — 38.0 kHz | 44 µs — 22.8 kHz | 59 µs — 16.9 kHz |
| `xarm7` | 7, fixed | 46 µs — 21.9 kHz | 84 µs — 11.8 kHz | 95 µs — 10.5 kHz |
| `go2` | 18, floating | 71 µs — 14.1 kHz | 126 µs — 8.0 kHz | 184 µs — 5.4 kHz |
| `g1` | 35, floating | 157 µs — 6.4 kHz | 353 µs — 2.8 kHz | 437 µs — 2.3 kHz |

`g1` is a Unitree G1, a **29-DOF humanoid**: 40 links, a floating base, and the
largest model in the suite. It closes a 6 kHz torque loop on a single
Cortex-M4F — and 3 kHz on the 80 MHz L413, inside 64 KB of RAM.

> [!NOTE]
> **Pick a core with a hardware FPU.** The two soft-float columns run 10–20×
> slower, so a quadruped wants an M4F, M33 or better. The Pico 2 image is the
> only one linked `copy_to_ram` and pays no flash wait states, which is most of
> what separates a 150 MHz M33 from a 170 MHz M4F here.
>
> The G474, L413 and ESP32-C6 columns are current. The two RP2350 columns were
> captured one release earlier, so their `rnea` and `gravity` read a few
> percent slow — on an FPU core, not on the Hazard3, where that change was
> worth nothing.

Raw CSV lives in [`benchmark/results/`](benchmark/results/), and
`tools/report.py` regenerates these tables from it.

---

## Against code generation

Pinocchio can trace RNEA, ABA and CRBA symbolically and let CasADi emit
straight-line C for one specific robot — no loops, no traversal, every
subexpression eliminated. That is the strongest opponent available, so
[`benchmark/codegen/`](benchmark/codegen/) measures against it. Both sides are
float32, `-O3`, same compiler, same board, and both are checked against
Pinocchio's own double-precision answer.

**Speed** — cycles per call on an STM32G474 at 170 MHz, `update_kinematics`
plus the algorithm against one generated call:

| | | RobotDynamics | Code generation | |
|---|---|---|---|---|
| `spine` | `rnea` | **4,843** | 12,498 | **2.6×** |
| 9 dof | `aba` | **10,089** | 27,211 | **2.7×** |
| | `crba` | **4,804** | 11,423 | **2.4×** |
| | `rnea` + `crba` | **7,805** | 23,927 | **3.1×** |
| `xarm7` | `rnea` | **8,312** | 25,354 | **3.1×** |
| 7 dof | `aba` | **21,094** | 30,671 | **1.5×** |
| | `crba` | **9,814** | 24,388 | **2.5×** |
| | `rnea` + `crba` | **15,011** | 49,743 | **3.3×** |
| `go2` | `rnea` | **13,058** | 58,120 | **4.5×** |
| 18 dof | `aba` | **31,526** | 79,780 | **2.5×** |
| | `crba` | **14,104** | 52,920 | **3.8×** |
| | `rnea` + `crba` | **22,419** | 111,042 | **5.0×** |

**The lead widens with the robot** — 2.4–3.1× on the 9-DOF spine, 3.8–5.0× on
Go2's `crba` and `rnea+crba` — because the generated code grows with the model
while the library does not.

**Size** — `.text` of the compiled objects, same compiler and flags:

| | Cortex-M4 code |
|---|---|
| Generated `rnea` + `aba` + `crba`, `spine` only | 17,425 bytes |
| … `xarm7` only | 29,021 bytes |
| … `go2` only | 67,221 bytes |
| … all three | 113,667 bytes |
| **RobotDynamics, whole library, any robot** | **40,236 bytes** |

An in-order core with 32 FP registers turns the generated code's thousands of
simultaneously-live temporaries into stack traffic, and the size makes every
iteration an instruction fetch. Code generation trades size for arithmetic, and
that trade pays on a desktop — on an x86-64 host it wins by 1.1–1.7×.

The same head-to-head on an STM32L413 is in
[`benchmark/results/`](benchmark/results/); its 128 KB of flash fits Go2's
generated code alone, where the G474's 512 KB fits all three robots at once.

---

## Licence

[Apache License 2.0](LICENSE). Free for any use, commercial included: ship it in
a product, modify it, keep your changes closed. The obligations are keeping the
licence and [NOTICE](NOTICE) with redistributions and stating what you changed.
Apache 2.0 also carries an explicit patent grant from every contributor, which
is the practical reason to prefer it for a library that may end up in a
commercial robot.

[bard](https://github.com/YueWang996/bard-pytorch-dynamics) is MIT, and the two
are compatible in either direction.

---

## Credit

This library was written for **[SPARC](https://github.com/YueWang996/sparc)**, a
3-DoF sagittal spine unit for quadruped robots: revolute pitch and prismatic
axial motion between the front and rear body segments, 1.26 kg, three
torque-controlled actuators with programmable compliance. A spine adds DOF to
the dynamics model and tightens the control loop at the same time, and that
pair of demands is the constraint this library was shaped around.

The design draws on **[bard](https://github.com/YueWang996/bard-pytorch-dynamics)**,
the PyTorch rigid-body dynamics library this one is the embedded counterpart
to. The model/data split, the algorithm set, the `q` ordering and the
spatial-algebra conventions all come from there, so a policy trained against
bard on a workstation runs against this library on the robot.

```bibtex
@inproceedings{wang2026batched,
  title         = {Batched Differentiable Rigid Body Dynamics in PyTorch for GPU-Accelerated Robot Learning},
  author        = {Wang, Yue and Xu, Yanran and Wu, Wenbo and Qiu, Chuanhang and Li, Zhaoxing},
  booktitle     = {International Conference on Artificial Neural Networks (ICANN)},
  year          = {2026},
  publisher     = {Springer},
  eprint        = {2605.31481},
  archivePrefix = {arXiv},
  url           = {https://arxiv.org/abs/2605.31481}
}
```

Correctness is measured against [Pinocchio](https://github.com/stack-of-tasks/pinocchio),
and the algorithms are Featherstone's.
