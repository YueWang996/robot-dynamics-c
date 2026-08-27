# Contacts and closed loops {#contacts}

The library has two interfaces for a robot touching the world. Which one to use
depends on whether the force or the motion is known.

| What you know | Use |
|---|---|
| The force (load cell reading, payload weight, thruster) | rd_rnea_ext(), rd_aba_ext() |
| The motion (a foot is planted and may not accelerate) | rd_constrained_dynamics() |
| Both (standing while carrying something) | rd_constrained_dynamics_ext() |

Choosing wrong gives plausible but incorrect results with no error.

## Known external forces

### Data format

`f_ext` is an array of length `6 * n_nodes`, one spatial force per link, in
**that link's own frame**:

```
f_ext[6*i + 0..2]   force, N
f_ext[6*i + 3..5]   torque, N·m
```

Links with no force are zero. Passing `NULL` for the whole array means no
external forces.

### Example

```c
static rd_real_t f_ext[6 * RD_MAX_LINKS];
memset(f_ext, 0, sizeof(f_ext));

/* Known world-frame force w[3] at the origin of link i. Rotate into the
   link frame: T holds a column-major rotation, so this is R transpose. */
rd_real_t T[16];
rd_forward_kinematics(&chain, &state, i, T);
f_ext[6*i + 0] = T[0]*w[0] + T[1]*w[1] + T[2]*w[2];
f_ext[6*i + 1] = T[4]*w[0] + T[5]*w[1] + T[6]*w[2];
f_ext[6*i + 2] = T[8]*w[0] + T[9]*w[1] + T[10]*w[2];

rd_rnea_ext(&chain, &state, qdd, NULL, f_ext, tau);
```

### Notes

Forces are applied inside the O(n) recursion, so each contact adds a few
additions. The common `tau += Jᵀ·f` formulation needs a Jacobian and a
transpose multiply per contact, which costs considerably more.

Feet are usually fixed links, and rd_chain_build() folds fixed links into their
nearest moving ancestor. Applying a force to a fixed link still works: the
array is indexed by link, and the fold carries the force to the moving ancestor
along with the inertia.

## Constraints: contacts and closed loops

### When to use

**Contact.** A foot on the ground. The contact force is unknown; what is known
is that the contact point may not accelerate. The force comes out of the solve.

**Closed loop.** A URDF describes a tree and cannot state that two links are
joined. The approach matches Pinocchio: the model keeps the tree, and the two
frames it was cut between are tied by a constraint.

Both use the same struct.

### Constraint types

```c
typedef enum {
    RD_CONSTRAINT_POINT = 0,   /* 3 rows: origins coincide, relative rotation free */
    RD_CONSTRAINT_FULL  = 1    /* 6 rows: origins coincide and no relative rotation */
} rd_constraint_type_t;

typedef struct {
    rd_idx_t             frame_a;   /* the constraint is written at this origin */
    rd_idx_t             frame_b;   /* the other end. RD_ANCHOR_WORLD for ground */
    rd_constraint_type_t type;
} rd_constraint_t;
```

@warning `frame_b` is an index, and the value for ground is `RD_ANCHOR_WORLD`,
which is `-1`. Writing `RD_FRAME_WORLD` compiles, but that is an enum of value
`0`, which as an index means link 0, the base. The result constrains the foot
to the body.

### Functions

```c
rd_int_t    rd_constrained_dynamics_work(const rd_chain_t* chain,
                                         const rd_constraint_t* cons, rd_int_t n_cons);

rd_status_t rd_constrained_dynamics(const rd_chain_t* chain, const rd_state_t* state,
                                    const rd_real_t* tau, const rd_real_t* gravity,
                                    const rd_constraint_t* cons, rd_int_t n_cons,
                                    rd_real_t* work,
                                    rd_real_t* qdd_out, rd_real_t* lambda_out);
```

| Parameter | Detail |
|---|---|
| `cons` | Array of constraints |
| `n_cons` | Number of constraints |
| `work` | Scratch array, sized by rd_constrained_dynamics_work() |
| `qdd_out` | Output joint acceleration, length `nv` |
| `lambda_out` | Output contact forces, length = total rows from rd_constraint_rows() |

Each constraint's entries in `lambda_out` are expressed at `frame_a`'s origin
in world-aligned axes.

### Example

```c
rd_constraint_t con[2] = {
    { fl_foot, RD_ANCHOR_WORLD, RD_CONSTRAINT_POINT },   /* front-left planted, 3 rows */
    { link_a,  link_b,          RD_CONSTRAINT_FULL  },   /* five-bar closed, 6 rows */
};

rd_int_t rows = rd_constraint_rows(con, 2);              /* = 9 */
rd_int_t nw   = rd_constrained_dynamics_work(&chain, con, 2);

static rd_real_t work[WORK_MAX];
static rd_real_t lambda[9];

rd_constrained_dynamics(&chain, &state, tau, NULL, con, 2, work, qdd, lambda);
```

Constraints are arguments and are not part of the chain. A leg lifting means a
shorter array on the next tick, with no rebuild and no allocation.

### The system solved

```
[ M   Jᵀ ] [  qdd  ]   [ tau - h ]
[ J   0  ] [ -λ    ] = [ -gamma  ]
```

`M` is factorised once and each constraint row costs one back-substitution.

### Constraint Jacobian and bias separately

```c
rd_int_t    rd_constraint_jacobian_work(const rd_chain_t* chain);   /* = 6*nv */

rd_status_t rd_constraint_jacobian(const rd_chain_t* chain, const rd_state_t* state,
                                   const rd_constraint_t* cons, rd_int_t n_cons,
                                   rd_real_t* work, rd_real_t* J_out);

rd_status_t rd_constraint_bias(const rd_chain_t* chain, const rd_state_t* state,
                               const rd_constraint_t* cons, rd_int_t n_cons,
                               rd_real_t* gamma_out);
```

`J_out` is `rows × nv` row-major and `gamma_out` has length `rows`.

Use these to assemble your own KKT system or to build a contact solver with
friction on top.

## Friction cone

rd_constrained_dynamics() solves an **equality** constrained system. It does not
know that the ground can only push, and it does not know that tangential force
has a limit. To satisfy a constraint it will produce a normal force pulling the
foot down and will ask for any tangential force it needs.

These checks are the caller's:

```c
/* For each point contact, lambda holds three components in world-aligned axes. */
rd_real_t fx = lambda[3*k+0], fy = lambda[3*k+1], fz = lambda[3*k+2];

if (fz < 0)                              /* normal force negative: foot being pulled down */
    /* the foot has actually left the ground; drop it and re-solve */;

if (fx*fx + fy*fy > mu*mu*fz*fz)         /* outside the friction cone */
    /* the tangential force cannot be delivered; redistribute or slow down */;
```

Enforcing these requires iteration, dropping the feet that fail and solving
again. Which iteration to use is a control decision and is not in this library.

## Self-check

Feeding the solved contact forces back through rd_rnea_ext() should reproduce
the input torques:

```c
rd_constrained_dynamics(&chain, &state, tau, NULL, con, n, work, qdd, lambda);

/* after converting lambda into f_ext */
rd_rnea_ext(&chain, &state, qdd, NULL, f_ext, tau_check);
/* tau_check should equal tau */
```

In `examples/go2_contact/` this round trip closes to 1.5e-05.

The check needs no reference implementation and runs on the target. It catches
sign errors and frame errors.

## Worked example

`examples/go2_contact/` runs the whole thing on a quadruped:

```bash
cd examples/go2_contact && make && ./go2_contact
```

It covers four-foot stance, force distribution, two-foot support, a friction
cone sweep and the self-check. Three results worth noting:

- With all four feet planted and zero joint torque, the body still falls at
  almost 1 g. The constraint keeps the feet still; standing up takes torque.
  The contact solve reports the ground reaction, which is 10 N of a 158 N
  weight.
- After splitting the weight evenly across the feet, `tau[0..5]` is non-zero,
  meaning the even split does not satisfy base equilibrium. Solving that
  residual is force distribution, usually a small QP.
- Torques must be recomputed when the support set changes. Four-foot torques
  applied with two feet down leave half the weight unsupported.

## Cost

STM32L413 @ 80 MHz, Go2, including rd_update_kinematics():

| | Time |
|---|---|
| No contacts, rd_forward_dynamics() with CRBA | 435 µs |
| Two point contacts | 1039 µs |

About 2.6×. The extra is the constraint Jacobian, the bias term and one
Cholesky back-substitution per constraint row, six of them for two point
contacts. The mass matrix is factorised once in both cases.
