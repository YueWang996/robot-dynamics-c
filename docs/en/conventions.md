# Conventions {#conventions}

Everything on this page produces a plausible wrong answer rather than an error.
None of it is checkable at run time, because a reversed quaternion and a
correct one are both unit quaternions.

The conventions match [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
and [bard](https://github.com/YueWang996/bard-pytorch-dynamics), so a `q`
vector written for one of them passes straight to this library and back.

## The short version

| | |
|---|---|
| Base pose | `q_base = [x y z qw qx qy qz]`, **scalar first** |
| Base twist and acceleration | The root link's **body frame** |
| Spatial vectors | `[linear, angular]` |
| `qd`, `qdd`, `tau` | Length `nv`, packed, base in the first six |
| `M` | `nv × nv`, fully filled |
| Joint order | Depth-first in URDF declaration order |
| Gravity | `NULL` means `{0, 0, -9.81}` in world axes |

## Configuration is split; velocity is not

`nq` and `nv` differ for a floating base, because a base pose needs a
quaternion to represent three rotational degrees of freedom. So configuration
arrives as two arguments and velocity as one:

```c
rd_real_t q_base[7];         /* x y z qw qx qy qz  -- scalar first          */
rd_real_t q_joints[nj];      /* one per actuated joint                      */
rd_real_t qd[nv];            /* nv = 6 + nj floating, nj fixed              */
```

For a fixed-base robot pass `NULL` for `q_base`; the base sits at the identity.
Passing `NULL` for `q_joints` gives the zero configuration, and `NULL` for `qd`
means at rest.

The scalar-first quaternion is the one to double check. ROS and Eigen both
write `[x y z w]`, and a quaternion loaded in the wrong order is still a unit
quaternion, so nothing anywhere will complain. What you get is a robot in an
attitude nobody asked for.

## The base twist lives in the root body frame

`qd[0..5]` is the base's spatial velocity expressed in the root link's own
frame, which is Pinocchio's free-flyer convention. It is not the world-frame
velocity of the base origin. If your state estimator hands you a world-frame
twist, rotate it into the root frame before it goes into `qd`.

The same applies to `qdd[0..5]` and to the first six entries of `tau`, which
are the net wrench on the base. A floating base has no actuator, so whatever
comes back in those six rows is the residual an equality that has not been met
yet. @ref contacts has a worked case of that.

## Spatial vectors are [linear, angular]

Six-element spatial quantities, and the rows of a Jacobian, order the linear
part first:

```
v = [vx vy vz  wx wy wz]
f = [fx fy fz  tx ty tz]
```

Featherstone's book puts angular first. This library follows Pinocchio.

## What "world frame" means

rd_frame_t has two values and RD_FRAME_WORLD is the one people misread.

- `RD_FRAME_LOCAL` — in the frame's own body axes, at its own origin.
- `RD_FRAME_WORLD` — Pinocchio's `ReferenceFrame.WORLD`: the spatial vector
  taken **at the world origin**, in world axes.

`RD_FRAME_WORLD` is not `LOCAL_WORLD_ALIGNED`. The two agree only when the
frame's origin sits on the world origin, and they differ by a term that grows
with how far out the frame is, which is why the mistake shows up as a small
error on a small robot and a large one on a leg.

To get the world-aligned quantity at a point `p`, take the WORLD one and shift
it:

```c
rd_jacobian(&chain, &state, frame, RD_FRAME_WORLD, J);
/* row-major, 6 x nv. Shift the linear rows to the point p: */
for (int c = 0; c < nv; ++c) {
    rd_real_t wx = J[3*nv + c], wy = J[4*nv + c], wz = J[5*nv + c];
    J[0*nv + c] -= p[1]*wz - p[2]*wy;
    J[1*nv + c] -= p[2]*wx - p[0]*wz;
    J[2*nv + c] -= p[0]*wy - p[1]*wx;
}
```

For accelerations there is one more term. The classical acceleration of a point
is the shifted spatial acceleration plus ω × ṗ; rd_spatial_acceleration()
returns the spatial one, and the cross term is the caller's to add.

## RD_ANCHOR_WORLD is not RD_FRAME_WORLD

These two read alike and mean unrelated things, and the compiler will not
separate them.

| | |
|---|---|
| RD_FRAME_WORLD | an rd_frame_t. Says which frame a Jacobian or velocity is *expressed in*. Its value is 0. |
| RD_ANCHOR_WORLD | an rd_idx_t. Says a constraint's second frame is *the world*. Its value is -1. |

Writing `RD_FRAME_WORLD` for a constraint's `frame_b` compiles, and quietly
constrains the foot to link 0 — the base — because that is what the enum's zero
means as an index. The foot then follows the body around and the solve returns
numbers that look reasonable.

## Joint order

Depth-first, in the order the URDF declares the joints, which is what Pinocchio
does. `tools/urdf2c.py` prints the mapping when it converts a model, and
rd_chain_find_frame() resolves a name to an index at run time.

Fixed links occupy a frame index but no velocity index. A foot is usually a
fixed link: it has an index you can ask for a Jacobian about, and it
contributes nothing to `nv`.

## The mass matrix is filled

rd_crba() writes the whole `nv × nv` matrix, both triangles, row-major. Passing
it to rd_cholesky_factor() works directly, and that function reads only the
lower triangle.

## Units

SI throughout. Metres, radians, kilograms, newtons, newton-metres, seconds.
Inertias are kg·m², and armature is in the units of a diagonal entry of `M`,
which for a revolute joint is kg·m².
