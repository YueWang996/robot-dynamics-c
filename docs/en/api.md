# API overview {#api}

Grouped by what you are trying to compute. Every name here links into the
[reference](files.html), which is generated from the headers.

## Setting up

Once, at startup. rd_chain_build() is the only function in the library that
allocates.

| | |
|---|---|
| rd_chain_build() | Turn an rd_model_t into the form the algorithms walk |
| rd_chain_free() | Give back what it allocated |
| rd_chain_find_frame() | Resolve a link name to a frame index |
| rd_chain_get_nv() | `nv`: the length of `qd`, `qdd` and `tau` |
| rd_chain_set_armature() | Reflected rotor inertia for one joint |
| rd_state_buffer_size() | Bytes of workspace an n-link model needs |
| rd_state_init() | Point an rd_state_t at that buffer |

## Once per tick

| | |
|---|---|
| rd_update_kinematics() | Transforms and velocities that everything else reads |

Call it first, then run as many algorithms as you like against the same state.
Skipping it after `q` changes gives you last tick's answer, silently.

## Kinematics

| | |
|---|---|
| rd_forward_kinematics() | A frame's 4×4 pose in the world, from the cache |
| rd_fk_frame() | The same pose without a state, walking the root-to-frame path |
| rd_jacobian() | Geometric Jacobian, `6 × nv` row-major |
| rd_spatial_velocity() | A frame's spatial velocity |
| rd_spatial_acceleration() | Its spatial acceleration, given `qdd` |

rd_fk_frame() takes `q_base` and `q_joints` rather than a state, so it answers
a "where would the foot be at this configuration" question without disturbing
the tick's cache. It costs a walk down one path; rd_forward_kinematics() costs
almost nothing and needs the cache to be current.

## Inverse dynamics

| | |
|---|---|
| rd_rnea() | `tau = M(q) qdd + C(q,qd) qd + g(q)` |
| rd_rnea_ext() | The same, with a known external force on any link |
| rd_gravity() | `g(q)` alone |
| rd_nonlinear_terms() | `C(q,qd) qd + g(q)`, the `h` in `M qdd + h` |
| rd_coriolis() | `C(q,qd) qd`, gravity removed |

## Mass matrix and forward dynamics

| | |
|---|---|
| rd_crba() | `M(q)`, `nv × nv`, both triangles filled |
| rd_aba() | `qdd` from `tau`, O(n), no mass matrix formed |
| rd_aba_ext() | The same with external forces |
| rd_forward_dynamics() | Either method, chosen by an argument |
| rd_forward_dynamics_work() | How much scratch that method wants |

Which of the two is faster is a property of the robot, and DOF count is not
what decides it. ABA's articulated-inertia congruence has a fast path that
needs a joint's origin to carry no rotation, and most URDFs give it that: Go2
takes it at all twelve joints. xarm7's origins are quarter turns, so it takes
the path at none of them, and that is why xarm7 is the one model where CRBA
wins. A floating base pushes the other way again, because its six DOF are
ancestors of every joint and leave the mass matrix with no sparsity for the
factorisation to use. @ref performance has the measured margins.

rd_aba() and rd_aba_ext() are compiled only when `RD_ENABLE_ABA` is 1, which is
the default. See @ref configuration for what turning it off buys.

## Linear algebra

| | |
|---|---|
| rd_cholesky_factor() | `L Lᵀ` of a symmetric positive definite matrix, in place |
| rd_cholesky_solve() | One solve against that factorisation |

Exposed because operational-space inertia, a task-space controller and a
constrained solve all want a factorised mass matrix and should not have to
bring their own. One factorisation serves any number of right-hand sides.

## Constraints and contacts

| | |
|---|---|
| rd_constraint_rows() | Rows a constraint set contributes: 3 per point, 6 per weld |
| rd_constraint_jacobian_work() | Scratch for the next one: `6*nv` |
| rd_constraint_jacobian() | The constraint Jacobian `J` |
| rd_constraint_bias() | The `gamma` that goes with it |
| rd_constrained_dynamics_work() | Scratch for the full solve |
| rd_constrained_dynamics() | `qdd` and the contact forces `lambda`, together |
| rd_constrained_dynamics_ext() | The same with a known external force as well |

@ref contacts is the page for these, including which one a planted foot wants.

## Types

| | |
|---|---|
| rd_model_t | A robot as data. `tools/urdf2c.py` emits one |
| rd_chain_t | That model prepared for traversal |
| rd_state_t | One tick's cache, plus every algorithm's scratch |
| rd_constraint_t | Two frames that must hold still relative to one another |
| rd_status_t | What every function returns. `RD_OK` is zero |
| rd_frame_t | Which frame a result is expressed in |
| rd_joint_type_t, rd_axis_t | What a joint does and about which axis |
| rd_real_t | `float` or `double`, by RD_USE_SINGLE_PRECISION |

## Who needs a scratch buffer

Most functions work entirely out of the rd_state_t. Four take an extra array,
and each has a sizing call next to it rather than a formula to remember:

| Function | Ask |
|---|---|
| rd_forward_dynamics() | rd_forward_dynamics_work() |
| rd_constraint_jacobian() | rd_constraint_jacobian_work() |
| rd_constrained_dynamics() | rd_constrained_dynamics_work() |
| rd_constrained_dynamics_ext() | rd_constrained_dynamics_work() |

The sizing calls are cheap and depend only on the chain and the constraint set,
so a control loop asks once at startup and declares the array statically.
