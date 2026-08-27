# Data formats {#conventions}

Vector layouts and frame definitions. These match
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) and
[bard](https://github.com/YueWang996/bard-pytorch-dynamics), so vectors pass
between them and this library directly.

@warning Nothing on this page produces an error when it is wrong. Functions
return `RD_OK` either way. Check each item during bring-up.

## Dimensions

| Symbol | Meaning | Value |
|---|---|---|
| `nj` | Actuated joints | `chain.n_joints` |
| `nv` | Velocity-space dimension | `6 + nj` floating, `nj` fixed. Use rd_chain_get_nv() |
| `n_nodes` | Total links, fixed links included | `chain.n_nodes` |

With a floating base the configuration dimension is one larger than the
velocity dimension, because attitude uses a quaternion: four numbers for three
degrees of freedom. The library splits configuration into two arguments so
callers do not handle the offset.

## Vector layout

| Variable | Length | Layout |
|---|---|---|
| `q_base` | 7 | `[x, y, z, qw, qx, qy, qz]` |
| `q_joints` | `nj` | One per actuated joint, in joint order |
| `qd` | `nv` | Floating base: first 6 are the base twist, then joint velocities |
| `qdd` | `nv` | Same as `qd` |
| `tau` | `nv` | Same as `qd`. The first 6 are the net wrench on the base |
| `M` | `nv × nv` | Row-major, both triangles filled |
| `J` | `6 × nv` | Row-major. First 3 rows linear, last 3 rows angular |
| `f_ext` | `6 × n_nodes` | One spatial force per link, in that link's own frame |
| `T` | 16 | 4×4 homogeneous transform, **column-major** |

## Quaternion order

`q_base[3..6]` is `[qw, qx, qy, qz]`, **scalar first**.

```c
q_base[0] = x;   q_base[1] = y;   q_base[2] = z;
q_base[3] = qw;  q_base[4] = qx;  q_base[5] = qy;  q_base[6] = qz;
```

ROS and Eigen use `[x, y, z, w]`, scalar last. A quaternion loaded in the wrong
order still has unit norm, so every check passes and the robot ends up in the
wrong attitude.

## Spatial vectors

Six-element velocities, accelerations and forces are spatial vectors. In this
library the **linear part comes first**:

```
velocity  v = [vx, vy, vz,  wx, wy, wz]
force     f = [fx, fy, fz,  tx, ty, tz]
```

The six rows of a Jacobian follow the same order. Featherstone's book puts the
angular part first, so formulas copied from it need reordering.

## Reference frames

rd_frame_t has two values:

| Value | Axes | Reference point |
|---|---|---|
| `RD_FRAME_LOCAL` | The frame's own | The frame's own origin |
| `RD_FRAME_WORLD` | World | **World origin** |

`RD_FRAME_WORLD` corresponds to Pinocchio's `ReferenceFrame.WORLD`. It is not
`LOCAL_WORLD_ALIGNED`. The two share axes and differ in reference point, and
they agree only when the frame's origin sits on the world origin.

Most applications want LOCAL_WORLD_ALIGNED: how fast and in which world
direction a point is moving. Convert from the `RD_FRAME_WORLD` result:

```c
/* J is the output of rd_jacobian(..., RD_FRAME_WORLD, J), row-major 6 x nv.
   p is the target point in world coordinates. */
for (int c = 0; c < nv; ++c) {
    rd_real_t wx = J[3*nv + c], wy = J[4*nv + c], wz = J[5*nv + c];
    J[0*nv + c] -= p[1]*wz - p[2]*wy;
    J[1*nv + c] -= p[2]*wx - p[0]*wz;
    J[2*nv + c] -= p[0]*wy - p[1]*wx;
}
```

Accelerations need one more term. The classical acceleration of a point is the
shifted spatial acceleration plus ω × ṗ. rd_spatial_acceleration() returns the
spatial acceleration; the cross term is the caller's.

## Frame of the base velocity

`qd[0..5]`, `qdd[0..5]` and `tau[0..5]` are all expressed in the **root link's
own frame**, matching Pinocchio's free-flyer convention.

A state estimator usually reports a world-frame twist. Rotate it into the root
frame before it goes into `qd`.

`tau[0..5]` is the net wrench on the base. A floating base has no actuator, so
non-zero values there indicate an equilibrium the current torques do not
satisfy. This is useful for checking whether force distribution has converged.

## Gravity

Every function taking a `gravity` argument treats `NULL` as `{0, 0, -9.81}` in
world axes.

The `rd_model_t::gravity` field is **not** used automatically. Pass it
explicitly:

```c
rd_rnea(&chain, &state, qdd, (const rd_real_t*)&my_robot.gravity, tau);
```

rd_vec3_t is three contiguous rd_real_t, so the cast is valid.

## Indices

| Kind | Detail |
|---|---|
| Frame index | `rd_idx_t`, `0` to `n_nodes - 1`. Get it from rd_chain_find_frame() |
| Velocity index | `rd_int_t`, `0` to `nv - 1`. Used by rd_chain_set_armature() |
| No parent | `rd_link_t::parent_idx` is `-1` |
| Constraint to ground | `rd_constraint_t::frame_b` is `RD_ANCHOR_WORLD`, value `-1` |

@warning `RD_ANCHOR_WORLD` and `RD_FRAME_WORLD` are different things.
`RD_ANCHOR_WORLD` is an `rd_idx_t` of value `-1` meaning the constraint's other
end is the world. `RD_FRAME_WORLD` is an `rd_frame_t` of value `0` naming a
reference frame. Writing `RD_FRAME_WORLD` for `frame_b` compiles and means
"constrain to link 0", which is the base.

Fixed links occupy a frame index but no velocity index. A foot is usually a
fixed link: you can take its Jacobian and constrain it, and it contributes
nothing to `nv`.

## Joint order

Depth-first in URDF declaration order, matching Pinocchio. `tools/urdf2c.py`
prints the full mapping during conversion. At run time, use
rd_chain_find_frame() to resolve names.

## Units

SI throughout.

| Quantity | Unit |
|---|---|
| Length | m |
| Angle | rad |
| Mass | kg |
| Force | N |
| Torque | N·m |
| Time | s |
| Inertia | kg·m² |
| armature | Same as a diagonal entry of `M`; kg·m² for a revolute joint |
