# Quick start {#quickstart}

From nothing to a running control loop.

## 1. Get the library

Download
[robot_dynamics.h](https://github.com/YueWang996/robot-dynamics-c/releases/latest)
into your project directory. That one file is the whole library.

In **exactly one** `.c` file, define `RD_IMPLEMENTATION` before including it.
The implementation compiles into that file:

```c
/* robot_dynamics_impl.c */
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

Every other file includes it plainly:

```c
#include "robot_dynamics.h"
```

## 2. Convert your model

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h --floating-base
```

Drop `--floating-base` for a fixed-base robot.

The output is a header containing `const rd_model_t my_robot = {...}`. Being
`const`, it goes in flash and costs no RAM.

The converter enforces three constraints and fails rather than approximating:

| Constraint | Detail |
|---|---|
| Axis-aligned joint axes | The URDF `<axis>` must be `±1 0 0`, `0 ±1 0` or `0 0 ±1`. A slanted axis has to be absorbed into the joint's `rpy` offset |
| Link names of 15 characters or fewer | Stored in `char name[16]` |
| Parents declared before children | Most URDFs already satisfy this |

## 3. Set the model size

Define two macros **before** including any header. They set the size of
rd_model_t:

```c
#define RD_MAX_LINKS   32
#define RD_MAX_JOINTS  12
#include "robot_dynamics.h"
```

Or on the compiler command line:

```bash
cc -DRD_MAX_LINKS=32 -DRD_MAX_JOINTS=12 ...
```

Defaults are 16 and 12. Counts for common robots are in @ref configuration.

## 4. Write the code

A complete, compiling example:

```c
#include "robot_dynamics.h"
#include "model_my_robot.h"

static rd_chain_t chain;
static rd_state_t state;
static rd_real_t  buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
static rd_idx_t   foot_id;

/* Joint-space vectors. nv = 6 + joint count for a floating base. */
static rd_real_t q_base[7], q_joints[RD_MAX_JOINTS];
static rd_real_t qd[6 + RD_MAX_JOINTS], qdd[6 + RD_MAX_JOINTS];
static rd_real_t tau[6 + RD_MAX_JOINTS];
static rd_real_t J[6 * (6 + RD_MAX_JOINTS)];

int setup(void)
{
    if (rd_chain_build(&my_robot, &chain) != RD_OK)
        return -1;

    if (rd_state_init(&state, chain.n_nodes, buf, sizeof(buf)) != RD_OK)
        return -1;

    foot_id = rd_chain_find_frame(&chain, "foot_fl");
    if (foot_id < 0)
        return -1;                       /* name does not match the model */

    return 0;
}

void control_tick(void)
{
    read_encoders(q_joints, qd);         /* your driver */
    read_imu(q_base, qd);                /* pass NULL for q_base if fixed base */

    /* Once per tick, before any other algorithm. */
    rd_update_kinematics(&chain, &state, q_base, q_joints, qd);

    /* Inverse dynamics: torques for the desired acceleration. */
    rd_rnea(&chain, &state, qdd, NULL, tau);

    /* Foot Jacobian, world frame. */
    rd_jacobian(&chain, &state, foot_id, RD_FRAME_WORLD, J);

    write_torques(tau);                  /* your driver */
}
```

Three ordering requirements:

1. rd_chain_build() first, once. It is the only function in the library that
   allocates from the heap.
2. rd_state_init() before `state` is used. It divides the buffer among the
   fields.
3. rd_update_kinematics() at the top of every tick. It computes the pose and
   velocity of every link, and the algorithms after it read that result. If
   `q` changes and you skip it, you get the previous tick's answer with no
   error.

Any number of algorithms may run in one tick against the same
rd_update_kinematics() result.

## 5. Compile

Single header, on a host:

```bash
cc -O2 -std=c99 main.c robot_dynamics_impl.c -lm -o app
```

CMake:

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

On an embedded target, enable hardware floating point. Without it the library
runs 6 to 19 times slower:

```bash
arm-none-eabi-gcc -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ...
```

## 6. Check the results

Read @ref conventions first. Quaternion order, reference frames and vector
layout produce no error when wrong, only wrong numbers.

Checks that ship with the library:

| Command | Purpose |
|---|---|
| `cmake -B build && cmake --build build && ./build/rd_test` | Host smoke test. Needs nothing installed |
| `python3 tools/validate.py --urdf-root /path/to/bard` | Full cross-check against Pinocchio. Needs Pinocchio |
| `python3 tools/amalgamate.py --verify` | Builds the single header and verifies its machine code matches the multi-file build |

## Common errors

| Symptom | Cause |
|---|---|
| rd_state_init() returns `RD_ERR_INVALID_SIZE` | Buffer too small. Ask rd_state_buffer_size() |
| rd_chain_build() returns `RD_ERR_INVALID_SIZE` | Model exceeds `RD_MAX_LINKS` or `RD_MAX_JOINTS` |
| rd_chain_find_frame() returns -1 | Link name misspelled, or truncated to 15 characters during conversion |
| A solver returns `RD_ERR_SINGULAR` | Mass matrix or constraint set is degenerate. Usually a duplicated constraint |
| Magnitudes right, attitude wrong | Quaternion written as `[x y z w]`. This library wants `[w x y z]` |
| All outputs zero | rd_update_kinematics() was not called |
| Torques too low | Geared joints without armature set. See @ref contacts |
