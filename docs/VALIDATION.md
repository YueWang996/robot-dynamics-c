# Validation against Pinocchio

Every algorithm in the library is checked against
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) on the same URDFs, for
randomly sampled configurations, in both `float64` and `float32` builds.

```bash
python3 tools/validate.py --urdf-root /path/to/bard --double -n 20   # exact
python3 tools/validate.py --urdf-root /path/to/bard -n 20            # shipping config
python3 tools/test_urdf2c.py --urdf-root /path/to/bard               # converter tests
```

`tools/validate.py` drives `tools/validate_dump.c`, which runs the C library and
prints its results at full precision; the Python side computes the same
quantities with Pinocchio and reports the worst relative error over all samples.

---

## Results

Twenty random configurations per robot, seeded for reproducibility. Error is
`max |a-b| / max(1, ‖a‖∞, ‖b‖∞)`.

### `float64` build — worst relative error

| Quantity | spine (9 dof, floating) | xarm7 (7 dof, fixed) | go2 (18 dof, floating) |
|---|---|---|---|
| Link world poses | 7.8e-16 | 1.7e-15 | 6.7e-16 |
| Link spatial velocities | 7.8e-16 | 3.0e-15 | 8.7e-16 |
| RNEA torques | 5.1e-16 | 1.9e-15 | 7.0e-16 |
| CRBA mass matrix | 3.7e-16 | 8.9e-16 | 2.8e-17 |
| Jacobian (world) | 7.8e-16 | 1.7e-15 | 5.6e-16 |
| Jacobian (local) | 5.6e-16 | 1.8e-15 | 5.6e-16 |
| Spatial acceleration | 9.5e-16 | 3.4e-15 | 5.7e-16 |

Machine precision throughout. The algorithms agree with Pinocchio exactly.

### `float32` build — worst relative error

| Quantity | spine | xarm7 | go2 |
|---|---|---|---|
| Link world poses | 1.8e-07 | 2.1e-07 | 1.9e-07 |
| Link spatial velocities | 3.0e-07 | 2.8e-07 | 2.7e-07 |
| RNEA torques | 3.8e-07 | 6.0e-07 | 2.6e-07 |
| CRBA mass matrix | 1.4e-07 | 1.3e-07 | 6.2e-08 |
| Jacobian (world) | 1.8e-07 | 2.1e-07 | 1.4e-07 |
| Jacobian (local) | 3.1e-07 | 2.8e-07 | 2.8e-07 |
| Spatial acceleration | 2.4e-07 | 3.9e-07 | 3.4e-07 |

`float32` epsilon is 1.19e-07, so the worst case across every quantity and every
robot is about five ULP. Crucially the error does **not** grow with model size —
Go2, with 31 links and the longest recursions, is no worse than the 4-link
spine. Nothing is being amplified through the tree traversal, which is the thing
that would actually threaten a single-precision implementation.

`simple_arm` is skipped: its URDF declares revolute joints with no `<limit>`
element, which urdfdom rejects outright. bard's own parser is more permissive.

---

## Convention mapping

Most of what a cross-check like this actually tests is whether two libraries
mean the same thing by the same vector. The mapping:

| | RobotDynamics | Pinocchio |
|---|---|---|
| Base quaternion | `[x y z qw qx qy qz]` — scalar **first** | `[x y z qx qy qz qw]` — scalar **last** |
| Base velocity frame | root body frame | root body frame (free-flyer) — same |
| Spatial vector order | `[linear, angular]` | `[linear, angular]` — same |
| Joint index order | depth-first in URDF joint order | depth-first in URDF joint order — same |
| Fixed-joint links | kept as zero-DOF nodes | folded into the parent body, kept as frames |
| Mass matrix | filled symmetric | upper triangle only; caller symmetrises |

The fixed-joint difference is why the link counts differ (Go2: 31 nodes here, 13
joints in Pinocchio) while the physics still matches exactly — folding a rigid
child's inertia into its parent is what CRBA's composite pass does anyway.

The quaternion ordering is worth calling out, since it is silent: feeding a
Pinocchio `q` straight in produces a plausible-looking but wrong pose.

---

## What this caught

The cross-check paid for itself immediately. Three of these were real defects,
one was a defect in the test itself.

### CRBA transformed composite inertia the wrong way

`rd_crba_cached` inverted `state->Ti` before accumulating a child's inertia into
its parent:

```c
rd_real_t T_cp[16];
rd_mat4_inv(&state->Ti[node*16], T_cp);
algo_transform_inertia_accumulate(T_cp, &Ic[node*36], &Ic[parent*36]);
```

The rule is `Ic_parent += (ᶜXₚ)ᵀ · Ic_child · ᶜXₚ`, where `ᶜXₚ` is the motion
transform from the parent frame into the child's — which is `Ad(Ti)`, and
`state->Ti` already holds exactly that. Inverting it computed the congruence in
the opposite direction.

Mass matrix error was **13% on spine and 40% on xarm7**; after the fix, 4e-16.
Removing the inversion also drops a `rd_mat4_inv` per link from the most
expensive algorithm in the library.

Notably, the smoke test's own symmetry and positive-diagonal checks passed
throughout — the wrong matrix was still symmetric and positive definite. Only a
reference implementation catches this class of bug.

### The converter's joint order did not match Pinocchio

`tools/urdf2c.py` emitted links breadth-first. On a serial chain that is
identical to depth-first, so spine and xarm7 passed; on Go2 it produced

```
q[0..3]  = the four hips
q[4..7]  = the four thighs
q[8..11] = the four calves
```

where Pinocchio (and therefore bard) uses leg-by-leg `FL_hip, FL_thigh,
FL_calf, FR_hip, …`. Nothing errors — a `q` vector just silently means
something else, which is the worst way for this to fail given the library's
whole premise is that you train against bard and deploy against this.

The converter now walks depth-first in URDF joint-declaration order, and every
generated header documents its own `q` mapping in the file header. A test in
`tools/test_urdf2c.py` asserts the order matches Pinocchio's for all three
robots, using a two-legged fixture that a serial-chain test would not catch.

### `RD_USE_SINGLE_PRECISION=0` did not actually give double precision

`rd_math.h` hardcoded `sinf`/`cosf`/`sqrtf`/`fabsf`/`atan2f` regardless of the
configured type, so a `double` build rounded every transcendental through
`float` and capped accuracy at ~1e-7. The helpers now dispatch on
`RD_REAL_IS_FLOAT`. Without this the `float64` column above would have read
1e-7 instead of 1e-16, and the single-precision analysis would have had no
baseline to be measured against.

### A too-strict tolerance in the converter's own test

`test_total_mass_matches_pinocchio` summed `model.inertias[1:]`, skipping index
0 — the universe body. xarm7 attaches `link_base` to the world through a fixed
joint *before* the first movable joint, so Pinocchio folds its 0.886 kg in
there. The converter was right; the test was wrong.

---

## Coverage and limits

What is checked: link world poses, link spatial velocities, RNEA torques, the
CRBA mass matrix, geometric Jacobians in both reference frames, and frame
spatial acceleration — on one fixed-base serial arm, one floating-base serial
chain, and one floating-base branched tree, at random configurations with
non-zero velocity and acceleration.

What is not:

- **`simple_arm`**, for the URDF reason above.
- **Prismatic joints.** All four benchmark robots are revolute-only. The code
  path exists and is exercised by the converter tests, but no numerical
  reference covers it.
- **Joint limits, damping and friction.** Parsed and stored, never used by any
  algorithm.
- **Forward dynamics.** ABA is not implemented, so there is nothing to compare.
- **`rd_gravity_compensation`**, which is a known-wrong entry point — see the
  README's limitations. It returns `C(q,q̇)q̇ + g(q)`, and the validation
  compares only `rd_rnea_cached`, which is correct.
