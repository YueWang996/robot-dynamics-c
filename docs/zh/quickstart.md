# 快速上手 {#quickstart}

## 拿到库

从 [Releases](https://github.com/YueWang996/robot-dynamics-c/releases/latest)
下载 **`robot_dynamics.h`**，扔进工程目录。一个文件，不用改构建系统，不用加子模块。

在其中一个 `.c` 文件里这样写，库的实现就编译进这个文件：

```c
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

只能有一个文件这么写。其他地方直接 `#include "robot_dynamics.h"` 就行。

发布的这个头文件和源码树是同一个库，这件事是验证过的。`tools/amalgamate.py --verify`
会把刚生成的头文件按七种配置各编译一遍，再把它产出的 Cortex-M4 机器码和多文件版本
逐个函数比对。40 个函数全部逐条指令一致。

## 从源码树用

如果你用 CMake，或者想自己生成那个单头文件：

```bash
git clone https://github.com/YueWang996/robot-dynamics-c
cd robot-dynamics-c
cmake -B build && cmake --build build && ./build/rd_test   # 主机上的冒烟测试
python3 tools/amalgamate.py --verify                       # 生成 dist/robot_dynamics.h
```

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

`./build/rd_test` 什么都不用装就能跑。
`python3 tools/validate.py --urdf-root /path/to/bard` 是完整的 Pinocchio 交叉验证，
那个需要先装 Pinocchio。

## 把你的机器人转进来

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h --floating-base
```

生成的是一个 `const` 初始化的 rd_model_t 结构体。因为是 `const`，编译器会把它放进
flash，不占 RAM。

转换器会检查几件 C 侧必须满足的事，遇到表达不了的模型直接报错停下，不会悄悄凑合：
关节轴必须和坐标轴平行、连杆名不超过 15 个字符、父连杆在数组里排在子连杆前面。

在 include 任何头文件之前，把 `RD_MAX_LINKS` 和 `RD_MAX_JOINTS` 改成你机器人的数量。
这两个宏决定 rd_model_t 结构体本身有多大，默认的 16 和 12 是按小机械臂配的。Go2
需要 31 个连杆、12 个关节。

## 一个控制周期长什么样

一共四个调用，前两个只在启动时做一次，后两个每周期做。

```c
#include "robot_dynamics.h"
#include "model_my_robot.h"

/* --- 启动时，各做一次 --- */
rd_chain_t chain;
rd_chain_build(&my_robot, &chain);          /* 把模型预处理成算法能走的形式 */

static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));   /* 划分工作区 */

const rd_idx_t eef = rd_chain_find_frame(&chain, "foot_fl");  /* 按名字查索引 */

/* --- 每个控制周期 --- */
for (;;) {
    read_encoders(q_joints, qd);
    read_base_estimate(q_base);             /* 固定基座的机器人传 NULL */

    rd_update_kinematics(&chain, &state, q_base, q_joints, qd);

    rd_rnea(&chain, &state, qdd, NULL, tau);            /* 逆动力学，算力矩 */
    rd_jacobian(&chain, &state, eef, RD_FRAME_WORLD, J);

    write_torques(tau);
}
```

rd_chain_build() 是库里唯一会向堆申请内存的函数，而且只在启动时申请一次。它之后所有
计算都在 `buf` 里做，这块内存归你，多大也由你定：rd_state_buffer_size() 告诉你要多少
字节，`RD_STATE_BUF_FLOATS(n)` 给的是静态数组声明用的元素个数。

rd_update_kinematics() 走一遍运动学树，把每根连杆的位姿和速度算出来存好，后面的算法
直接读。每个周期开头调一次，然后想跑几个算法就跑几个。为两个算法各调一次，等于把整个
控制环里最贵的那一步白算了一遍。

## 各个函数算什么

```c
rd_rnea(&chain, &state, qdd, NULL, tau);          /* tau = M qdd + h    */
rd_crba(&chain, &state, M);                       /* 质量矩阵 M(q)      */
rd_gravity(&chain, &state, NULL, g);              /* 只要重力项 g(q)    */
rd_nonlinear_terms(&chain, &state, NULL, h);      /* 科氏力加重力 C qd + g */
rd_forward_dynamics(&chain, &state, tau, NULL,
                    RD_FD_CRBA, work, qdd);       /* 由力矩反求加速度   */
rd_jacobian(&chain, &state, frame, RD_FRAME_WORLD, J);
rd_forward_kinematics(&chain, &state, frame, T);  /* frame 在世界系的位姿 */
```

这几个调用里的 `NULL` 都是重力参数。传 `NULL` 表示用世界系下的 `{0, 0, -9.81}`；
你的机器人如果在斜坡上或者别的星球上，传三个浮点数进去。

@warning `rd_forward_dynamics()` 要你额外给一块临时数组，多大取决于选哪个方法。
别猜，问 rd_forward_dynamics_work()：选 RD_FD_ABA 时它返回 0，选 RD_FD_CRBA 时返回
`nv*nv + nv`（一个质量矩阵加一个向量的空间）。

## 返回值

所有函数都返回 rd_status_t，`RD_OK` 是 0，错误码都是负数。

调试阶段这些码有用。rd_state_init() 返回 `RD_ERR_INVALID_SIZE` 说明缓冲区给小了；
求解函数返回 `RD_ERR_SINGULAR` 说明质量矩阵或者约束集退化了，常见原因是同一个约束
写了两遍。

控制环跑起来之后，每周期传的参数形状都一样，返回值也就一直一样。多数固件在启动时
查一遍，然后把检查从热路径上拿掉。

## 接下来

在相信第一组数字之前，先看一遍 @ref conventions "约定"。那页上列的每一条弄错了都
不会报错，程序照常跑，只是算出来的机器人是错的。
