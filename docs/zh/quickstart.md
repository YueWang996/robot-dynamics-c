# 快速上手 {#quickstart}

## 拿到库

从 [Releases](https://github.com/YueWang996/robot-dynamics-c/releases/latest)
下载 **`robot_dynamics.h`**，扔进你的工程。一个文件，不用构建系统，不用子模块。
在恰好一个 `.c` 文件里：

```c
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

其他地方直接 include 就行。

发布的那个头文件和源码树是同一个库，而且这件事是验证过的，不是声称的：
`tools/amalgamate.py --verify` 会把刚生成的头文件按多种配置编译一遍，再逐个函数
比对它和多文件构建产出的 Cortex-M4 代码。全部 40 个函数逐条指令一致。

## 从源码树用

CMake 工程，或者你想自己生成单头文件：

```bash
git clone https://github.com/YueWang996/robot-dynamics-c
cd robot-dynamics-c
cmake -B build && cmake --build build && ./build/rd_test   # 主机冒烟测试
python3 tools/amalgamate.py --verify                       # dist/robot_dynamics.h
```

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

`./build/rd_test` 不需要装任何东西。
`python3 tools/validate.py --urdf-root /path/to/bard` 跑完整的 Pinocchio 交叉验证，
那个需要装 Pinocchio。

## 把机器人拿进来

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h --floating-base
```

出来的是一个 `const` 初始化的 rd_model_t，所以模型待在 flash 里，不占 RAM。
转换器会检查 C 侧模型的硬性要求，遇到表达不了的东西就直接报错停下：关节轴必须与
坐标轴对齐，连杆名不超过 15 个字符，父连杆排在子连杆前面。

在 include 任何东西之前，把 `RD_MAX_LINKS` 和 `RD_MAX_JOINTS` 设成你的机器人需要的
数值。它们决定 rd_model_t 本身的大小，默认的 16 和 12 是按小机械臂配的；Go2 需要
31 个连杆、12 个关节。

## 控制环长什么样

四个调用，只有后两个是每周期的。

```c
#include "robot_dynamics.h"
#include "model_my_robot.h"

rd_chain_t chain;
rd_chain_build(&my_robot, &chain);          /* 启动时一次 */

static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));

const rd_idx_t eef = rd_chain_find_frame(&chain, "foot_fl");

for (;;) {
    read_encoders(q_joints, qd);
    read_base_estimate(q_base);             /* 固定基座传 NULL */

    rd_update_kinematics(&chain, &state, q_base, q_joints, qd);

    rd_rnea(&chain, &state, qdd, NULL, tau);            /* 逆动力学 */
    rd_jacobian(&chain, &state, eef, RD_FRAME_WORLD, J);

    write_torques(tau);
}
```

rd_chain_build() 是库里唯一会分配内存的函数，而且只分配一次。它之后的一切都在 `buf`
里干活，这块内存由你持有、你定大小：rd_state_buffer_size() 给字节数，
`RD_STATE_BUF_FLOATS(n)` 给静态声明用的元素个数。

rd_update_kinematics() 走一遍树，后面的算法读它留下的东西。每个周期开头调一次，
然后想跑多少算法就跑多少，都对着同一个 state。为两个算法调两次，等于把整个环里最贵的
那部分白算一遍。

## 每个调用给你什么

```c
rd_rnea(&chain, &state, qdd, NULL, tau);          /* tau = M qdd + h    */
rd_crba(&chain, &state, M);                       /* M(q)，nv x nv      */
rd_gravity(&chain, &state, NULL, g);              /* 单独的 g(q)        */
rd_nonlinear_terms(&chain, &state, NULL, h);      /* C qd + g           */
rd_forward_dynamics(&chain, &state, tau, NULL,
                    RD_FD_CRBA, work, qdd);       /* 由 tau 求 qdd      */
rd_jacobian(&chain, &state, frame, RD_FRAME_WORLD, J);
rd_forward_kinematics(&chain, &state, frame, T);  /* 该 frame 在世界系的位姿 */
```

这些调用里的 `NULL` 是重力。传 NULL 表示世界系下的 `{0, 0, -9.81}`；要换成别的，
传三个浮点数进去。

@warning `rd_forward_dynamics()` 需要一块临时数组，大小取决于用哪个方法。
问 rd_forward_dynamics_work()，别猜：RD_FD_ABA 不需要，RD_FD_CRBA 需要 `nv*nv + nv`。

## 检查返回值

每个函数都返回 rd_status_t，`RD_OK` 是 0。调试期这些码值得读：rd_state_init() 返回
`RD_ERR_INVALID_SIZE` 说明缓冲区不够，求解返回 `RD_ERR_SINGULAR` 说明质量矩阵或者
约束集退化了。

环跑起来之后，每个周期的参数都一样，返回值也一样。多数固件在启动时查一次，然后把检查
从热路径上拿掉。

## 下一步

在相信第一组数字之前，先读 @ref conventions "约定"。那页上的每一条弄错了都不会报错，
只会给你一个看起来合理的错误答案。
