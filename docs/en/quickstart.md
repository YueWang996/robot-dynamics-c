# Quick start {#quickstart}

## Get the library

Download **`robot_dynamics.h`** from
[Releases](https://github.com/YueWang996/robot-dynamics-c/releases/latest) and
drop it into your project. One file, no build system, no submodule. In exactly
one `.c` file:

```c
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

and include it plainly everywhere else.

That released header is the same library as the source tree, and it is checked
rather than asserted: `tools/amalgamate.py --verify` compiles the header it
just wrote several ways and compares its Cortex-M4 code against the multi-file
build, function by function. All 40 of them come out instruction for instruction
identical.

## Working from the tree

For a CMake project, or to build the single header yourself:

```bash
git clone https://github.com/YueWang996/robot-dynamics-c
cd robot-dynamics-c
cmake -B build && cmake --build build && ./build/rd_test   # host smoke test
python3 tools/amalgamate.py --verify                       # dist/robot_dynamics.h
```

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

`./build/rd_test` needs nothing installed.
`python3 tools/validate.py --urdf-root /path/to/bard` runs the full Pinocchio
cross-check, and wants Pinocchio.

## Bring in a robot

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h --floating-base
```

Out comes an rd_model_t as a `const` initialiser, so the model lives in flash
and costs no RAM. The converter enforces what the C model requires and fails
loudly on anything it cannot represent: joint axes have to be axis-aligned,
link names fit in 15 characters, and parents come before children.

Set `RD_MAX_LINKS` and `RD_MAX_JOINTS` to what your robot needs before you
include anything. They size rd_model_t itself, so the defaults of 16 and 12 are
sized for a small arm; Go2 needs 31 links and 12 joints.

## The shape of a control loop

Four calls, and only the last two are per tick.

```c
#include "robot_dynamics.h"
#include "model_my_robot.h"

rd_chain_t chain;
rd_chain_build(&my_robot, &chain);          /* once, at startup */

static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));

const rd_idx_t eef = rd_chain_find_frame(&chain, "foot_fl");

for (;;) {
    read_encoders(q_joints, qd);
    read_base_estimate(q_base);             /* NULL for a fixed base */

    rd_update_kinematics(&chain, &state, q_base, q_joints, qd);

    rd_rnea(&chain, &state, qdd, NULL, tau);            /* inverse dynamics */
    rd_jacobian(&chain, &state, eef, RD_FRAME_WORLD, J);

    write_torques(tau);
}
```

rd_chain_build() is the only function in the library that allocates, and it
does it once. Everything after it works out of `buf`, which you own and size
yourself: rd_state_buffer_size() gives the bytes, `RD_STATE_BUF_FLOATS(n)` the
element count for a static declaration.

rd_update_kinematics() walks the tree, and the algorithms after it read what it
left behind. Call it once at the top of the tick, then run as many algorithms
as you like against the same state. Calling it twice for two algorithms doubles
the most expensive part of the loop for nothing.

## What each call gives you

```c
rd_rnea(&chain, &state, qdd, NULL, tau);          /* tau = M qdd + h    */
rd_crba(&chain, &state, M);                       /* M(q), nv x nv      */
rd_gravity(&chain, &state, NULL, g);              /* g(q) alone         */
rd_nonlinear_terms(&chain, &state, NULL, h);      /* C qd + g           */
rd_forward_dynamics(&chain, &state, tau, NULL,
                    RD_FD_CRBA, work, qdd);       /* qdd from tau       */
rd_jacobian(&chain, &state, frame, RD_FRAME_WORLD, J);
rd_forward_kinematics(&chain, &state, frame, T);   /* pose in the world */
```

The `NULL` in those calls is gravity. Passing NULL means `{0, 0, -9.81}` in
world axes; pass three floats to say something else.

@warning `rd_forward_dynamics()` wants a scratch array, and how much depends on
the method. Ask rd_forward_dynamics_work() rather than guessing: RD_FD_ABA
needs none, RD_FD_CRBA needs `nv*nv + nv`.

## Checking the return value

Every function returns rd_status_t, and `RD_OK` is zero. During bring-up the
codes are worth reading: `RD_ERR_INVALID_SIZE` from rd_state_init() means the
buffer is short, and `RD_ERR_SINGULAR` from a solve means the mass matrix or
the constraint set is degenerate.

Once the loop is running, the arguments are the same every tick and so is the
answer. Most firmware checks at startup and drops the check from the hot path.

## Next

@ref conventions is the page to read before trusting the first numbers. Every
item on it produces a plausible wrong answer rather than an error.
