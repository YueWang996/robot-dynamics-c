# Contact dynamics for a quadruped

A worked example on a Unitree Go2: four feet, a trot, and the friction cone.

```bash
make && ./go2_contact
```

Needs nothing but the library and `benchmark/models/model_go2.h`, which is
generated from the URDF that ships with
[bard](https://github.com/YueWang996/bard-pytorch-dynamics).

## The one thing to get right

A planted foot is a **constraint**, not a force you know. Two different calls,
and picking the wrong one is the usual way to get plausible numbers that are
wrong:

| | |
|---|---|
| `rd_rnea_ext` | a force you already know and are applying — a measured load cell, a payload, a thruster |
| `rd_constrained_dynamics` | a foot that is planted and must not accelerate; its force comes back in `lambda` |
| `rd_constrained_dynamics_ext` | both at once, which is what a robot carrying anything needs |

## What the example shows

**A constraint holds the foot, not the robot.** Plant all four feet with zero
joint torque and the body still falls at almost a full g — the feet do not
move, the knees fold, and the ground supplies 10 N of a 158 N weight. Standing
up is a torque problem; the contact only says what the ground gives back.

**Choosing the foot forces is the caller's job.** Split the weight evenly, ask
`rd_rnea_ext` for the torques that hold still under those forces, and the first
six rows come back non-zero: the floating base has no actuator, so what is left
there is the equilibrium an even split failed to reach. Driving it to zero is
force distribution — a small QP in general, a control decision, and
deliberately not in this library. Send the torques anyway and the base holds
its height to 0.06 m/s² and pitches at 5.8 rad/s², which is exactly that
residual showing up as motion.

**The stance set and the torques move together.** Sending four-foot torques
with two feet down leaves half the weight unsupported. Recompute the split for
the feet that are actually down and the ground carries all 158 N again.
Constraints are an argument, so a gait is an array that gets shorter and
longer — nothing is rebuilt and nothing is allocated.

**The friction cone is where this becomes contact dynamics.** Ask the feet to
shove the robot sideways and the tangential demand climbs with the
acceleration while the normal force stays pinned by the weight. Past μ·g it
cannot be delivered whatever the torques say. `rd_constrained_dynamics` solves
an *equality* constrained system: it will pull a foot down if that is what
holding the constraint takes, and ask for any tangential force it likes.
Checking that the normal force is positive and the tangential one inside the
cone is left to the caller, because enforcing it is an iteration — release the
feet that fail and solve again — and which iteration is a control decision.

**Closing the loop.** The forces from the forward solve, fed back through
`rd_rnea_ext`, reproduce the torques it started from to 1.5e-05. That check
needs no reference implementation, and it is where a wrong sign or a wrong
frame shows up.

## Cost

STM32L413 at 80 MHz, Go2, `update_kinematics` + the solve:

| | |
|---|---|
| no contacts (`rd_forward_dynamics`, CRBA) | 435 µs |
| two point contacts | 1039 µs |

About 2.6×. The extra is the constraint Jacobian, the bias, and one Cholesky
back-substitution per constraint row. See the top-level README for the rest of
the platforms.
