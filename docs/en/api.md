# Functions {#api}

Grouped by purpose. Each entry gives the signature, what it computes, a
parameter table and an example. Full type definitions are in the
[API reference](files.html).

Every function returns rd_status_t. `RD_OK` is 0 and errors are negative.

## Setup

The following are called once, at startup.

### rd_chain_build

```c
rd_status_t rd_chain_build(const rd_model_t* model, rd_chain_t* chain);
```

Preprocesses a model into the form the algorithms traverse: topological order,
each link's inertia shifted to its own origin, fixed links folded into their
nearest moving ancestor, and joints marked as eligible for the fast kernels.

| Parameter | Detail |
|---|---|
| `model` | The model produced by `tools/urdf2c.py` |
| `chain` | Output. The struct is the caller's; the pointers inside it are allocated here |

| Return | Meaning |
|---|---|
| `RD_ERR_ALLOC_FAILED` | Out of heap |
| `RD_ERR_INVALID_SIZE` | Model exceeds `RD_MAX_LINKS` or `RD_MAX_JOINTS` |

This is the only function in the library that calls `malloc`. Release it with
rd_chain_free().

### rd_state_buffer_size and rd_state_init

```c
size_t      rd_state_buffer_size(rd_int_t n_nodes);
rd_status_t rd_state_init(rd_state_t* state, rd_int_t n_nodes,
                          void* buffer, size_t buffer_size);
```

Divides a caller-provided buffer among the algorithms' workspaces. Neither
function allocates or frees.

| Parameter | Detail |
|---|---|
| `n_nodes` | Link count. Use `chain.n_nodes` |
| `buffer` | At least `rd_state_buffer_size(n_nodes)` bytes, aligned for rd_real_t |
| `buffer_size` | Actual size in bytes, used for validation |

For a static declaration use `RD_STATE_BUF_FLOATS(n)`, which gives an element
count:

```c
static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));
```

Each link costs 70 rd_real_t, or 45 with `RD_ENABLE_ABA=0`.

### rd_chain_find_frame

```c
rd_idx_t rd_chain_find_frame(const rd_chain_t* chain, const char* name);
```

Resolves a link name to a frame index. Returns `-1` if no link has that name.
A linear scan, so resolve the frames you need at startup and keep the indices.

Every link in the model is findable, fixed links included.

## Per tick

### rd_update_kinematics

```c
rd_status_t rd_update_kinematics(const rd_chain_t* chain, rd_state_t* state,
                                 const rd_real_t* q_base,
                                 const rd_real_t* q_joints,
                                 const rd_real_t* qd);
```

Computes the pose and spatial velocity of every link and stores them in
`state`. **Call once at the top of every control tick.** The algorithms after
it read this result.

| Parameter | Detail |
|---|---|
| `q_base` | Length 7, `[x y z qw qx qy qz]`. `NULL` for a fixed base |
| `q_joints` | Length `nj`. `NULL` means the zero configuration |
| `qd` | Length `nv`. `NULL` means at rest |

Any number of algorithms may run against one call. Skipping it produces the
previous tick's data with no error.

## Kinematics

### rd_forward_kinematics

```c
rd_status_t rd_forward_kinematics(const rd_chain_t* chain, const rd_state_t* state,
                                  rd_idx_t frame_id, rd_real_t T_out[16]);
```

A frame's pose in the world as a 4×4 homogeneous transform, **column-major**.

Composed from the cache, so it is cheap. Requires rd_update_kinematics() to
have run this tick.

```c
rd_real_t T[16];
rd_forward_kinematics(&chain, &state, foot_id, T);
rd_real_t px = T[12], py = T[13], pz = T[14];   /* position is the 4th column */
```

### rd_fk_frame

```c
rd_status_t rd_fk_frame(const rd_chain_t* chain,
                        const rd_real_t* q_base, const rd_real_t* q_joints,
                        rd_idx_t frame_id, rd_real_t T_out[16]);
```

The same world pose, taking a configuration directly instead of a `state`.

Use it to answer hypothetical questions: where a frame would be at some other
configuration. Collision prediction and inverse-kinematics iteration need this.
It does not disturb the current tick's cache.

Costs a walk down the root-to-frame path, so it is more expensive than
rd_forward_kinematics().

### rd_jacobian

```c
rd_status_t rd_jacobian(const rd_chain_t* chain, const rd_state_t* state,
                        rd_idx_t frame_id, rd_frame_t ref_frame,
                        rd_real_t* J_out);
```

Geometric Jacobian, `6 × nv` row-major. First 3 rows linear, last 3 angular.

| Parameter | Detail |
|---|---|
| `ref_frame` | `RD_FRAME_WORLD` or `RD_FRAME_LOCAL`. See @ref conventions |
| `J_out` | Output, length `6 * nv` |

```c
static rd_real_t J[6 * (6 + RD_MAX_JOINTS)];
rd_jacobian(&chain, &state, foot_id, RD_FRAME_WORLD, J);

/* Column k: what degree of freedom k contributes to this frame. */
rd_real_t col_k[6];
for (int r = 0; r < 6; ++r) col_k[r] = J[r*nv + k];
```

### rd_spatial_velocity and rd_spatial_acceleration

```c
rd_status_t rd_spatial_velocity(const rd_chain_t* chain, const rd_state_t* state,
                                rd_idx_t frame_id, rd_frame_t ref_frame,
                                rd_real_t v_out[6]);

rd_status_t rd_spatial_acceleration(const rd_chain_t* chain, const rd_state_t* state,
                                    const rd_real_t* qdd,
                                    rd_idx_t frame_id, rd_frame_t ref_frame,
                                    rd_real_t a_out[6]);
```

A frame's spatial velocity and acceleration, `[linear, angular]`.

The acceleration call takes `qdd`. It returns the spatial acceleration;
converting to a point's classical acceleration needs ω × ṗ added. See
@ref conventions.

## Inverse dynamics

Motion is known, torques are computed.

### rd_rnea

```c
rd_status_t rd_rnea(const rd_chain_t* chain, const rd_state_t* state,
                    const rd_real_t* qdd, const rd_real_t* gravity,
                    rd_real_t* tau_out);
```

`tau = M(q)·qdd + C(q,q̇)·q̇ + g(q)`, an O(n) recursion that never forms the mass
matrix.

| Parameter | Detail |
|---|---|
| `qdd` | Desired joint acceleration, length `nv`. `NULL` means zero |
| `gravity` | World-frame gravity, length 3. `NULL` means `{0, 0, -9.81}` |
| `tau_out` | Output torques, length `nv` |

```c
/* Gravity compensation plus a desired acceleration. */
rd_rnea(&chain, &state, qdd_desired, NULL, tau);

/* Gravity and Coriolis only. */
rd_rnea(&chain, &state, NULL, NULL, tau);
```

With a floating base, `tau_out[0..5]` is the net wrench on the base, where
there is no actuator.

### rd_rnea_ext

```c
rd_status_t rd_rnea_ext(const rd_chain_t* chain, const rd_state_t* state,
                        const rd_real_t* qdd, const rd_real_t* gravity,
                        const rd_real_t* f_ext, rd_real_t* tau_out);
```

Same as rd_rnea(), with known external forces on links.

| Parameter | Detail |
|---|---|
| `f_ext` | Length `6 * n_nodes`, one spatial force per link in **that link's own frame**. `NULL` is equivalent to rd_rnea() |

Forces are applied inside the O(n) recursion, so each contact costs a few adds.
Usage is in @ref contacts.

### rd_gravity, rd_nonlinear_terms, rd_coriolis

```c
rd_status_t rd_gravity(const rd_chain_t* chain, const rd_state_t* state,
                       const rd_real_t* gravity, rd_real_t* tau_out);

rd_status_t rd_nonlinear_terms(const rd_chain_t* chain, const rd_state_t* state,
                               const rd_real_t* gravity, rd_real_t* tau_out);

rd_status_t rd_coriolis(const rd_chain_t* chain, const rd_state_t* state,
                        rd_real_t* tau_out);
```

| Function | Computes | Use |
|---|---|---|
| rd_gravity() | `g(q)` | Gravity compensation |
| rd_nonlinear_terms() | `C(q,q̇)·q̇ + g(q)`, the `h` term | Assembling `M·qdd + h = tau` |
| rd_coriolis() | `C(q,q̇)·q̇` | When the Coriolis term is needed on its own |

rd_gravity() runs the RNEA recursion with the cached velocities suppressed, so
it gives `g(q)` without rebuilding the state at `q̇ = 0`.

## Forward dynamics

Torques are known, acceleration is computed.

### rd_forward_dynamics

```c
rd_int_t    rd_forward_dynamics_work(const rd_chain_t* chain, rd_fd_method_t method);

rd_status_t rd_forward_dynamics(const rd_chain_t* chain, const rd_state_t* state,
                                const rd_real_t* tau, const rd_real_t* gravity,
                                rd_fd_method_t method,
                                rd_real_t* work, rd_real_t* qdd_out);
```

| Parameter | Detail |
|---|---|
| `tau` | Joint torques, length `nv`. `NULL` means zero |
| `method` | `RD_FD_ABA` or `RD_FD_CRBA` |
| `work` | Scratch array, sized by rd_forward_dynamics_work() |
| `qdd_out` | Output acceleration, length `nv` |

`work` length: 0 for `RD_FD_ABA`, `nv*nv + nv` for `RD_FD_CRBA`.

```c
static rd_real_t work[NV_MAX*NV_MAX + NV_MAX];
rd_forward_dynamics(&chain, &state, tau, NULL, RD_FD_CRBA, work, qdd);
```

**Choosing a method.** Both give the same result. Which is faster depends on
the robot:

| Situation | Use |
|---|---|
| Floating base, joint origins carry no rotation (most URDFs) | `RD_FD_ABA` |
| Fixed-base arm with rotated joint origins | `RD_FD_CRBA` |
| Many degrees of freedom (nv > 20) with a floating base | `RD_FD_ABA` |
| The mass matrix is wanted anyway | `RD_FD_CRBA`, which produces `M` |
| RAM is tight | `RD_FD_ABA`, which needs no `work` |

Measurements are in @ref performance. The gap ranges from 4% to 47%, so measure
your own model.

### rd_aba and rd_aba_ext

```c
rd_status_t rd_aba(const rd_chain_t* chain, const rd_state_t* state,
                   const rd_real_t* tau, const rd_real_t* gravity,
                   rd_real_t* qdd_out);

rd_status_t rd_aba_ext(const rd_chain_t* chain, const rd_state_t* state,
                       const rd_real_t* tau, const rd_real_t* gravity,
                       const rd_real_t* f_ext, rd_real_t* qdd_out);
```

The articulated-body algorithm directly, with no `work` array. `f_ext` has the
same format as in rd_rnea_ext().

Compiled only when `RD_ENABLE_ABA` is 1, which is the default.

### rd_crba

```c
rd_status_t rd_crba(const rd_chain_t* chain, const rd_state_t* state,
                    rd_real_t* M_out);
```

The mass matrix `M(q)`, `nv × nv` row-major, both triangles filled.

```c
static rd_real_t M[NV_MAX * NV_MAX];
rd_crba(&chain, &state, M);
```

Diagonal entries include the armature of any joint that has one set.

## Linear algebra

### rd_cholesky_factor and rd_cholesky_solve

```c
rd_status_t rd_cholesky_factor(rd_real_t* A, rd_int_t n, rd_real_t* dinv);

rd_status_t rd_cholesky_solve(const rd_real_t* L, const rd_real_t* dinv,
                              const rd_real_t* b, rd_real_t* x, rd_int_t n);
```

`L·Lᵀ` factorisation and solve for a symmetric positive definite matrix.

| Parameter | Detail |
|---|---|
| `A` | Input matrix, row-major. **Overwritten in place** with `L` in the lower triangle; the upper triangle is untouched |
| `dinv` | Output of length `n` holding `1/L_ii`, needed by the solve |
| `b`, `x` | Right-hand side and solution, length `n`. They may not alias |

Only the lower triangle of `A` is read, so rd_crba() output can be passed
straight in. `RD_ERR_SINGULAR` means the matrix is not positive definite.

One factorisation serves any number of right-hand sides, which is how
`J·M⁻¹·Jᵀ` is computed:

```c
rd_crba(&chain, &state, M);
rd_cholesky_factor(M, nv, dinv);            /* factor once */

for (int r = 0; r < 6; ++r)                 /* one solve per row */
    rd_cholesky_solve(M, dinv, &J[r*nv], &MinvJt[r*nv], nv);
```

## Gearing

### rd_chain_set_armature

```c
rd_status_t rd_chain_set_armature(rd_chain_t* chain, rd_int_t vidx, rd_real_t value);
```

Sets a joint's reflected rotor inertia, `n²·I_rotor` where `n` is the gear
ratio.

| Parameter | Detail |
|---|---|
| `vidx` | Velocity index: the joint's position in `qd`/`qdd`/`tau` |
| `value` | Same units as a diagonal entry of `M`; kg·m² for a revolute joint |

URDF has no field for this, so a converted model starts at zero. Set it after
rd_chain_build():

```c
rd_chain_build(&my_robot, &chain);
for (int j = 0; j < chain.n_joints; ++j)
    rd_chain_set_armature(&chain, 6 + j, gear_ratio*gear_ratio*rotor_inertia);
```

Once set, rd_crba() adds it to the diagonal of `M`, rd_rnea() adds it to `tau`,
and rd_aba() adds it to the articulated inertia, which keeps the three
consistent.

On a servo or any high-ratio drive the reflected inertia is often larger than
the link being driven. Leaving it at zero makes the computed torques too low.

## Release

### rd_chain_free

```c
void rd_chain_free(rd_chain_t* chain);
```

Releases what rd_chain_build() allocated. The rd_chain_t struct itself is left
alone and may be rebuilt.

Any rd_state_t used with this chain holds stale data afterwards and needs
rd_update_kinematics() before it is read again.
