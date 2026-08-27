# RobotDynamics {#mainpage}

Rigid-body dynamics for embedded targets. C99, distributed as a single header,
no dependencies beyond `libm`.

On an STM32G474 (Cortex-M4F @ 170 MHz), one torque computation for an 18-DOF
quadruped takes 71 µs, which is a 14.1 kHz control rate.

## Features

### Algorithms

| Feature | Functions |
|---|---|
| Forward kinematics | rd_forward_kinematics(), rd_fk_frame() |
| Geometric Jacobian | rd_jacobian() |
| Spatial velocity and acceleration | rd_spatial_velocity(), rd_spatial_acceleration() |
| Inverse dynamics (RNEA) | rd_rnea(), rd_rnea_ext() |
| Forward dynamics (ABA) | rd_aba(), rd_aba_ext() |
| Forward dynamics (CRBA + Cholesky) | rd_forward_dynamics() |
| Mass matrix (CRBA) | rd_crba() |
| Gravity term | rd_gravity() |
| Coriolis and nonlinear terms | rd_coriolis(), rd_nonlinear_terms() |
| Constrained dynamics (contacts, closed loops) | rd_constrained_dynamics(), rd_constrained_dynamics_ext() |
| Constraint Jacobian and bias | rd_constraint_jacobian(), rd_constraint_bias() |
| Cholesky factor and solve | rd_cholesky_factor(), rd_cholesky_solve() |

### Supported models

| | |
|---|---|
| Joint types | Revolute, prismatic, fixed |
| Base | Fixed base, floating base (6 DOF) |
| Joint axes | Must be axis-aligned: ±X / ±Y / ±Z |
| Closed loops | Supported, declared as constraints |
| Contacts | Supported, solved as equality constraints |
| Gearing | Reflected rotor inertia (armature) per joint |
| Model input | URDF, converted offline by `tools/urdf2c.py` |

### Numerics and resources

| | |
|---|---|
| Precision | float32 by default, float64 optional |
| Validation | Against Pinocchio: 1.9e-06 in float32, 5.5e-15 in float64 |
| Allocation | Only rd_chain_build(), once at startup. Zero allocation in the control loop |
| Code size | 40.2 KB of Cortex-M4 code for the whole library, about 25 KB after the linker drops unused parts |
| Minimum target | Runs on Cortex-M0+. A single-precision FPU (M4F or M33) is recommended |
| Thread safety | One algorithm at a time per rd_state_t. Use one state per thread |

## Not included

| | Alternative |
|---|---|
| Controllers | Write your own. The library supplies M, h, J and the factorisation |
| Friction cone / LCP contact solvers | The library solves equality constraints. Cone checks and iteration are the caller's |
| Collision detection | Use a separate library |
| Joint limit, damping and friction dynamics | The fields are parsed and stored. No algorithm reads them yet |
| Runtime URDF parsing | Convert offline with `tools/urdf2c.py` |

## Install

**Option 1, single header.** Download `robot_dynamics.h` from
[Releases](https://github.com/YueWang996/robot-dynamics-c/releases/latest) and
copy it into your project. In exactly one `.c` file:

```c
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

Include it plainly everywhere else.

**Option 2, source tree with CMake.**

```bash
git clone https://github.com/YueWang996/robot-dynamics-c
```

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

## Documentation

| | |
|---|---|
| @subpage quickstart | Convert a model and write a working program |
| @subpage conventions | Vector layouts, quaternion order, frames, units |
| @subpage api | Every function: signature, parameters, example |
| @subpage contacts | Feet on the ground, closed loops, known external forces |
| @subpage configuration | All compile-time macros and CMake options |
| @subpage performance | Measured results on five boards |

The [API reference](files.html) is generated from the headers.

## Licence

Apache License 2.0. Commercial use and closed-source modification are allowed.
Keep the licence and NOTICE with redistributions and state what you changed.
Includes an explicit patent grant.

## Related projects

- [bard](https://github.com/YueWang996/bard-pytorch-dynamics) — PyTorch
  implementation. Same `q` layout and spatial-algebra conventions, so vectors
  pass between the two directly
- [SPARC](https://github.com/YueWang996/sparc) — the original application, a
  3-DOF sagittal spine unit for quadrupeds
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — reference
  implementation used for validation
