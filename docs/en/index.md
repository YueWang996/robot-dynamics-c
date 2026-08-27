# RobotDynamics {#mainpage}

*Rigid-body dynamics that fits on a microcontroller.*

C99, `libm` and nothing else, and a 1 kHz torque loop for an 18-DOF quadruped
on a $5 board.

Give it a model derived from a URDF and one buffer, and it returns torques,
Jacobians and mass matrices fast enough to close the loop on the robot itself.
There is no controller here and no contact solver: you get M, h, J, J̇q̇ and a
factorisation, and what you build on them is yours.

## Where to start

| | |
|---|---|
| @subpage quickstart | Get the header, write the first control tick |
| @subpage conventions | Reversed, these produce a wrong robot quietly |
| @subpage api | Every function, grouped by what you are computing |
| @subpage configuration | Build options, and bringing your own sin/cos |
| @subpage contacts | Feet on the ground, closed chains, external forces |
| @subpage performance | Measured on five boards, and the smallest part it runs on |

The [API reference](files.html) is generated from the headers and is in English
on both sites. Everything above is written in both.

## What is in it

| | |
|---|---|
| Algorithms | Forward kinematics, geometric Jacobian, RNEA, ABA, CRBA, gravity, Coriolis and nonlinear terms, spatial velocity and acceleration |
| Joints | Revolute, prismatic, fixed, and a floating base |
| Frames | World and body, wherever a reference frame applies |
| Contacts | External spatial forces on any link, applied inside the O(n) recursion |
| Constraints | Feet on the ground and loops the tree was cut at, solved as a KKT system |
| Actuators | Reflected rotor inertia per joint, which any geared drive needs |
| Linear algebra | The Cholesky the forward dynamics uses, exposed for task-space work |
| Accuracy | Validated against Pinocchio at 5.5e-15 (float64) and 1.9e-06 (float32) |
| Precision | `float` by default, `double` with one flag |
| Allocation | None in the control loop. One caller-provided buffer holds every algorithm's scratch |
| Dependencies | `libm`. That is the whole list |
| Footprint | 40.2 KB of Cortex-M4 code for the whole library, about 25 KB once `--gc-sections` drops what you do not call |
| Distribution | One header |
| Model input | URDF, through `tools/urdf2c.py` |

A control tick usually wants several of these at once, and each of them starts
by walking the kinematic tree. rd_update_kinematics() walks it once and
everything afterwards reads that cache, which is where most of the speed comes
from.

@note Pre-1.0, with three gaps worth stating. Prismatic joints are implemented
and unvalidated, because the reference robots are revolute-only. Joint limits,
damping and friction are parsed and stored, and no algorithm reads them yet.
rd_chain_build() allocates once at startup, so a genuinely heapless build is
not available; the control loop is allocation-free either way.

## Licence

Apache License 2.0. Free for any use, commercial included: ship it in a
product, modify it, keep your changes closed. The obligations are keeping the
licence and NOTICE with redistributions and stating what you changed. Apache
2.0 also carries an explicit patent grant from every contributor, which is the
practical reason to prefer it for a library that may end up in a commercial
robot.

## Credit

Written for [SPARC](https://github.com/YueWang996/sparc), a 3-DoF sagittal
spine unit for quadruped robots. A spine adds DOF to the dynamics model and
tightens the control loop at the same time, and that pair of demands is the
constraint this library was shaped around.

The design draws on [bard](https://github.com/YueWang996/bard-pytorch-dynamics),
the PyTorch rigid-body dynamics library this one is the embedded counterpart
to. The model/data split, the algorithm set, the `q` ordering and the
spatial-algebra conventions all come from there, so a policy trained against
bard on a workstation runs against this library on the robot.

Correctness is measured against
[Pinocchio](https://github.com/stack-of-tasks/pinocchio), and the algorithms
are Featherstone's.
