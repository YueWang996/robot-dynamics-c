# Contacts and closed chains {#contacts}

There are two senses of "contact" in this library and they do different jobs.
Picking the wrong one is the usual way to get plausible numbers that are wrong.

| | |
|---|---|
| rd_rnea_ext(), rd_aba_ext() | a force you already know and are applying: a measured load cell, a payload, a thruster |
| rd_constrained_dynamics() | a foot that is planted and must not accelerate. Its force comes back in `lambda` |
| rd_constrained_dynamics_ext() | both at once, which is what a robot carrying anything needs |

## A force you already know

rd_rnea_ext() and rd_aba_ext() take one spatial force per **link**, in that
link's own body frame, and fold it into the O(n) recursion. A contact therefore
costs a few adds rather than a Jacobian transpose per contact.

```c
rd_real_t f_ext[6 * RD_MAX_LINKS] = {0};
/* A world-frame force w on link i, at that link's origin: */
rotate_world_to_body(&T_i[0], w, &f_ext[6*i]);
rd_rnea_ext(&chain, &state, qdd, NULL, f_ext, tau);
```

Placing one on a foot works even though feet are usually fixed links that
rd_chain_build() folded away: the array is indexed by link, and the fold
carries the force to the moving ancestor along with the inertia.

Links with no force cost six comparisons each, and passing `NULL` costs
nothing at all.

## A foot that is planted

A planted foot is a **constraint**. You do not know its force; you know its
acceleration is zero, and the force is what comes out.

A URDF is a tree and cannot say a robot has a loop in it, so a loop is declared
the way Pinocchio declares one: keep the tree the model already is, and
constrain the two frames it was cut between. The same object pins a foot to the
ground, with the world as the second frame.

```c
rd_constraint_t con[2] = {
    { fl_foot, RD_ANCHOR_WORLD, RD_CONSTRAINT_POINT },   /* a foot planted   */
    { link_a,  link_b,          RD_CONSTRAINT_FULL  },   /* a five-bar closed */
};

rd_int_t nw = rd_constrained_dynamics_work(&chain, con, 2);
static rd_real_t work[NW_MAX];
rd_real_t lambda[6];

rd_constrained_dynamics(&chain, &state, tau, NULL, con, 2,
                        work, qdd, lambda);
```

Constraints are arguments rather than part of the chain, because a legged
robot's contact set changes every time a foot leaves the ground. A leg lifting
is a different array on the next tick, and nothing is rebuilt or allocated.

@warning `frame_b` is an index, and the value that means "the world" is
RD_ANCHOR_WORLD, which is `-1`. Writing RD_FRAME_WORLD there compiles, because
that is an rd_frame_t whose value is `0`, and quietly constrains the foot to
link 0 — the base. The foot then follows the body around and the solve returns
numbers that look reasonable. See @ref conventions.

## What it solves

The KKT system, with M factorised once:

```
[ M   Jᵀ ] [  qdd  ]   [ tau - h ]
[ J   0  ] [ -λ    ] = [ -gamma  ]
```

`J` is the constraint Jacobian and `gamma` the bias that keeps the constraint
satisfied at the acceleration level. rd_constraint_jacobian() and
rd_constraint_bias() are public too, for anyone assembling their own KKT or
building a contact solver on top.

Rows are written in world-aligned axes at `frame_a`'s origin, and `lambda`
comes back in the same axes. Three rows per RD_CONSTRAINT_POINT, six per
RD_CONSTRAINT_FULL; rd_constraint_rows() adds them up.

## Four things the Go2 example shows

`examples/go2_contact/` works this end to end on a quadruped. `make &&
./go2_contact`, and it needs nothing but the library and a generated Go2 model.

**A constraint holds the foot, not the robot.** Plant all four feet with zero
joint torque and the body still falls at almost a full g. The feet do not move,
the knees fold, and the ground supplies 10 N of a 158 N weight. Standing up is
a torque problem; the contact only says what the ground gives back.

**Choosing the foot forces is the caller's job.** Split the weight evenly, ask
rd_rnea_ext() for the torques that hold still under those forces, and the first
six rows come back non-zero — the floating base has no actuator, so what is
left there is the equilibrium an even split failed to reach. Driving it to zero
is force distribution: a small QP in general, a control decision, and
deliberately not in this library. Send the torques anyway and the base holds
its height to 0.06 m/s² and pitches at 5.8 rad/s², which is exactly that
residual showing up as motion.

**The stance set and the torques move together.** Sending four-foot torques
with two feet down leaves half the weight unsupported. Recompute the split for
the feet that are actually down and the ground carries all 158 N again.

**The friction cone is where this becomes contact dynamics.**
rd_constrained_dynamics() solves an *equality* constrained system. It will pull
a foot down if that is what holding the constraint takes, and it will ask for
any tangential force it likes. Ask the feet to shove the robot sideways and the
tangential demand climbs with the acceleration while the normal force stays
pinned by the weight; past μ·g it cannot be delivered whatever the torques say.
Checking that the normal force is positive and the tangential one inside the
cone is left to the caller, because enforcing it is an iteration — release the
feet that fail and solve again — and which iteration is a control decision.

## Closing the loop on yourself

The forces from a forward solve, fed back through rd_rnea_ext(), reproduce the
torques it started from. In the Go2 example that round trip closes to 1.5e-05.

This check needs no reference implementation and no Pinocchio, and it is where
a wrong sign or a wrong frame shows up. Worth wiring into your own bring-up.

## Cost

STM32L413 at 80 MHz, Go2, `update_kinematics` plus the solve:

| | |
|---|---|
| no contacts (`rd_forward_dynamics`, CRBA) | 435 µs |
| two point contacts | 1039 µs |

About 2.6×. The extra is the constraint Jacobian, the bias, and one Cholesky
back-substitution per constraint row — six of them for two point contacts. The
mass matrix is factorised once either way.

## Geared joints

Not a contact, and it belongs in the same conversation because it changes the
same equations. `armature` is the reflected rotor inertia, `n²·I_rotor`, which
on a servo or any high-ratio drive is routinely larger than the link it turns.

URDF has no field for it, so a converted model starts at zero and
rd_chain_set_armature() fills it in. It is added to `M`'s diagonal by rd_crba(),
to `tau` by rd_rnea(), and to the articulated inertia by rd_aba(), which keeps
the three consistent with each other. A model without it skips the work: walking
the diagonal costs 3–5% of rd_crba(), so the chain records whether any joint has
one.
