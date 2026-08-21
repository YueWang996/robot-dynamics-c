<div align="center">

# RobotDynamics

**Rigid-body dynamics that fits on a microcontroller.**

C99 · `libm` and nothing else · a 1 kHz torque loop for an 18-DOF quadruped on a $5 board

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
| **Accuracy** | Validated against Pinocchio at **5.5e-15** (float64) and **1.9e-06** (float32) |
| **Precision** | `float` by default, `double` with one flag |
| **Allocation** | None in the control loop. One caller-provided buffer holds every algorithm's scratch |
| **Dependencies** | `libm`. That is the whole list |
| **Footprint** | **24.8 KB** of Cortex-M4 code for the entire library |
| **Model input** | URDF, through `tools/urdf2c.py` |
| **Interop** | `q`, `qd`, `qdd` and `tau` use Pinocchio's layout, so vectors pass straight to [bard](https://github.com/YueWang996/bard-pytorch-dynamics) and back |

A control tick usually wants several quantities at once, and each of them
begins by walking the kinematic tree. `rd_update_kinematics` walks it once and
every algorithm afterwards reads that cache, which is where most of the speed
comes from.

> [!NOTE]
> Pre-1.0, with three gaps worth stating. Prismatic joints are implemented and
> unvalidated, since all four reference robots are revolute-only. Joint limits,
> damping and friction are parsed and stored, and no algorithm reads them yet.
> `rd_chain_build()` allocates once at startup, so `RD_STATIC_ALLOC=ON` is not
> supported; the control loop is allocation-free either way.

---

## Using it

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

**Build it:**

```bash
cmake -B build && cmake --build build && ./build/rd_test
```

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

**Bring in a robot:**

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h --floating-base
```

The converter enforces what the C model requires — axis-aligned joint axes,
15-character link names, parents before children — and fails loudly on anything
it cannot represent.

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
| `update_kinematics` | 27.4 | 55.5 | 24.9 | 265.3 | 355.3 |
| `fk_frame` | 10.6 | 22.2 | 9.9 | 107.0 | 142.3 |
| `jacobian_world` | 12.3 | 25.4 | 10.3 | 63.8 | 81.1 |
| `jacobian_local` | 19.9 | 41.6 | 16.7 | 177.6 | 224.3 |
| `rnea` | 51.5 | 100.6 | 45.4 | 767.9 | 936.1 |
| `crba` | 60.9 | 113.3 | 55.9 | 530.4 | 678.5 |
| `aba` | 181.0 | 327.1 | 120.2 | 1756.8 | 2208.1 |
| `gravity` | 34.5 | 72.0 | 32.7 | 314.3 | 474.6 |
| `spatial_acceleration` | 22.8 | 40.8 | 17.9 | 202.8 | 275.9 |
| `spatial_velocity` | 4.7 | 9.7 | 4.6 | 52.3 | 72.5 |
| | | | | | |
| **torque tick** | 79 µs<br>12.7 kHz | 156 µs<br>6.4 kHz | 70 µs<br>14.2 kHz | 1033 µs<br>968 Hz | 1291 µs<br>774 Hz |
| **operational space** | 140 µs<br>7.2 kHz | 269 µs<br>3.7 kHz | 126 µs<br>7.9 kHz | 1564 µs<br>640 Hz | 1970 µs<br>508 Hz |
| **forward dynamics** | 208 µs<br>4.8 kHz | 383 µs<br>2.6 kHz | 145 µs<br>6.9 kHz | 2022 µs<br>495 Hz | 2563 µs<br>390 Hz |

Torque tick is `update_kinematics` + `rnea`; operational space adds `crba`;
forward dynamics is `update_kinematics` + the faster of ABA and CRBA for that
robot.

How it scales with the robot, on the slowest FPU part here — an STM32L413 at
80 MHz:

| Robot | nv | Torque tick | Operational space | Forward dynamics |
|---|---|---|---|---|
| `spine` | 9 | 58 µs — 17.3 kHz | 94 µs — 10.6 kHz | 122 µs — 8.2 kHz |
| `xarm7` | 7 | 100 µs — 10.0 kHz | 181 µs — 5.5 kHz | 206 µs — 4.9 kHz |
| `go2` | 18 | 156 µs — 6.4 kHz | 269 µs — 3.7 kHz | 383 µs — 2.6 kHz |

> [!NOTE]
> **Pick a core with a hardware FPU.** The two soft-float columns run 10–20×
> slower, so a quadruped wants an M4F, M33 or better. The Pico 2 image is the
> only one linked `copy_to_ram` and pays no flash wait states, which is why a
> 150 MHz M33 lands ahead of a 170 MHz M4F. The G474 column is four changes
> behind the others; that board is away.

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

**Speed** — Go2 on an STM32L413 at 80 MHz, cycles per call, `update_kinematics`
plus the algorithm against one generated call:

| | RobotDynamics | Code generation | |
|---|---|---|---|
| `rnea` | **13,161** | 50,729 | **3.9×** |
| `aba` | **35,042** | 72,215 | **2.1×** |
| `crba` | **14,696** | 44,823 | **3.1×** |
| `rnea` + `crba` | **23,239** | 95,534 | **4.1×** |

**Size** — Go2's three generated algorithms against this library in full:

| | Cortex-M4 code |
|---|---|
| Generated `rnea` + `aba` + `crba`, Go2 only | 66,534 bytes |
| **RobotDynamics, whole library, any robot** | **24,794 bytes** |

An in-order core with 32 FP registers turns the generated code's thousands of
simultaneously-live temporaries into stack traffic, and the size makes every
iteration an instruction fetch: the generated code pays a 25–29% fetch tax
where this library's RNEA and CRBA pay 7–9%. Code generation trades size for
arithmetic, and that trade pays on a desktop — on an x86-64 host it wins by
1.1–1.7×.

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
