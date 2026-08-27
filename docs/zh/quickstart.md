# 快速开始 {#quickstart}

本页从零走到一个能跑的控制循环。

## 1. 获取库

下载 [robot_dynamics.h](https://github.com/YueWang996/robot-dynamics-c/releases/latest)
放进工程目录。整个库就这一个文件。

在**恰好一个** `.c` 文件里定义 `RD_IMPLEMENTATION` 再包含它，实现会编译进这个文件：

```c
/* robot_dynamics_impl.c */
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

其余文件直接包含即可：

```c
#include "robot_dynamics.h"
```

## 2. 转换模型

```bash
python3 tools/urdf2c.py my_robot.urdf -n my_robot -o model_my_robot.h --floating-base
```

固定基座的机器人去掉 `--floating-base`。

输出是一个 `const rd_model_t my_robot = {...}` 的头文件，编译后放在 flash，不占 RAM。

转换器的限制，不满足会直接报错：

| 限制 | 说明 |
|---|---|
| 关节轴与坐标轴平行 | URDF 的 `<axis>` 只能是 `±1 0 0` / `0 ±1 0` / `0 0 ±1`。倾斜的轴需要吸收进关节的 `rpy` 偏置 |
| 连杆名不超过 15 字符 | 名字存在 `char name[16]` 里 |
| 父连杆排在子连杆前面 | URDF 一般已满足 |

## 3. 设置模型规模

在包含任何头文件**之前**定义两个宏，它们决定 rd_model_t 结构体的大小：

```c
#define RD_MAX_LINKS   32
#define RD_MAX_JOINTS  12
#include "robot_dynamics.h"
```

或者在编译命令行上给：

```bash
cc -DRD_MAX_LINKS=32 -DRD_MAX_JOINTS=12 ...
```

默认值是 16 和 12。常见机器人的数值见 @ref configuration "编译选项"。

## 4. 写代码

完整可编译的例子：

```c
#include "robot_dynamics.h"
#include "model_my_robot.h"

static rd_chain_t chain;
static rd_state_t state;
static rd_real_t  buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
static rd_idx_t   foot_id;

/* 关节空间向量。浮动基座时 nv = 6 + 关节数 */
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
        return -1;                       /* 名字拼错了 */

    return 0;
}

void control_tick(void)
{
    read_encoders(q_joints, qd);         /* 你的驱动 */
    read_imu(q_base, qd);                /* 固定基座时 q_base 传 NULL */

    /* 每周期一次，必须在其他算法之前 */
    rd_update_kinematics(&chain, &state, q_base, q_joints, qd);

    /* 逆动力学：由期望加速度算力矩 */
    rd_rnea(&chain, &state, qdd, NULL, tau);

    /* 脚的雅可比，世界系 */
    rd_jacobian(&chain, &state, foot_id, RD_FRAME_WORLD, J);

    write_torques(tau);                  /* 你的驱动 */
}
```

调用顺序有三条硬性要求：

1. rd_chain_build() 在最前，只调一次。它是库里唯一分配堆内存的函数。
2. rd_state_init() 在使用 `state` 之前调用，把缓冲区划分给各个字段。
3. rd_update_kinematics() 在每个周期的开头调一次。它算出所有连杆的位姿和速度，
   后面的算法读这份结果。`q` 变了却没调它，得到的是上一周期的结果，且不报错。

一个周期里可以调任意多个算法，它们共用同一次 rd_update_kinematics() 的结果。

## 5. 编译

单头文件，主机上：

```bash
cc -O2 -std=c99 main.c robot_dynamics_impl.c -lm -o app
```

CMake：

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

嵌入式目标记得开硬件浮点，否则慢 6 到 19 倍：

```bash
arm-none-eabi-gcc -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ...
```

## 6. 验证结果

先读 @ref conventions "数据格式"。四元数顺序、参考系、向量排布这几项弄错了程序不会
报错，只是结果不对。

库自带的检查手段：

| 命令 | 作用 |
|---|---|
| `cmake -B build && cmake --build build && ./build/rd_test` | 主机冒烟测试，不需要装任何东西 |
| `python3 tools/validate.py --urdf-root /path/to/bard` | 对照 Pinocchio 的完整交叉验证，需要装 Pinocchio |
| `python3 tools/amalgamate.py --verify` | 生成单头文件并验证它与多文件构建产出的机器码一致 |

## 常见错误

| 现象 | 原因 |
|---|---|
| rd_state_init() 返回 `RD_ERR_INVALID_SIZE` | 缓冲区小了。用 rd_state_buffer_size() 问需要多少字节 |
| rd_chain_build() 返回 `RD_ERR_INVALID_SIZE` | 模型超过 `RD_MAX_LINKS` 或 `RD_MAX_JOINTS` |
| rd_chain_find_frame() 返回 -1 | 连杆名拼错，或者被 URDF 转换时截断到 15 字符 |
| 求解函数返回 `RD_ERR_SINGULAR` | 质量矩阵或约束集退化，常见原因是同一条约束写了两遍 |
| 结果数量级正确但姿态离谱 | 四元数写成了 `[x y z w]`。本库要求 `[w x y z]` |
| 结果全零 | 忘了调 rd_update_kinematics() |
| 力矩偏小 | 用了减速箱但没设 armature，见 @ref contacts "接触与闭链" |
