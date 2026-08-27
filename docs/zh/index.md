# RobotDynamics {#mainpage}

面向嵌入式的刚体动力学库。C99，单头文件分发，除 `libm` 外无依赖。

在 STM32G474（Cortex-M4F @ 170 MHz）上，18 自由度四足机器人算一轮力矩耗时
71 µs，对应 14.1 kHz 控制频率。

## 功能

### 算法

| 功能 | 函数 |
|---|---|
| 正运动学 | rd_forward_kinematics()、rd_fk_frame() |
| 几何雅可比 | rd_jacobian() |
| 空间速度、空间加速度 | rd_spatial_velocity()、rd_spatial_acceleration() |
| 逆动力学（RNEA） | rd_rnea()、rd_rnea_ext() |
| 正动力学（ABA） | rd_aba()、rd_aba_ext() |
| 正动力学（CRBA + Cholesky） | rd_forward_dynamics() |
| 质量矩阵（CRBA） | rd_crba() |
| 重力项 | rd_gravity() |
| 科氏项、非线性项 | rd_coriolis()、rd_nonlinear_terms() |
| 约束动力学（接触、闭链） | rd_constrained_dynamics()、rd_constrained_dynamics_ext() |
| 约束雅可比与偏置项 | rd_constraint_jacobian()、rd_constraint_bias() |
| Cholesky 分解与回代 | rd_cholesky_factor()、rd_cholesky_solve() |

### 支持的模型

| | |
|---|---|
| 关节类型 | 转动、移动、固定 |
| 基座 | 固定基座、浮动基座（6 自由度） |
| 关节轴 | 必须与坐标轴平行，即 ±X / ±Y / ±Z |
| 闭链 | 支持，以约束形式声明 |
| 接触 | 支持，按等式约束求解 |
| 减速比 | 支持折算转子惯量（armature） |
| 模型来源 | URDF，用 `tools/urdf2c.py` 离线转成 C 结构体 |

### 数值与资源

| | |
|---|---|
| 精度 | 默认 float32，可切 float64 |
| 验证 | 对照 Pinocchio，float32 误差 1.9e-06，float64 误差 5.5e-15 |
| 内存分配 | 仅 rd_chain_build() 在启动时分配一次，控制环内零分配 |
| 代码体积 | 整库 40.2 KB（Cortex-M4），按需链接后约 25 KB |
| 最低硬件 | Cortex-M0+ 可运行，推荐带单精度 FPU 的 M4F 或 M33 |
| 线程安全 | 一个 rd_state_t 同时只能有一个算法在跑；多个线程各用各的 state |

## 不包含的功能

| | 替代方案 |
|---|---|
| 控制器 | 自行实现，本库提供 M、h、J 和分解结果 |
| 摩擦锥、LCP 接触求解 | 本库解等式约束，摩擦锥检查与迭代由调用方做 |
| 碰撞检测 | 自行实现或使用其他库 |
| 关节限位、阻尼、摩擦的动力学 | 字段会解析并保存在模型里，目前没有算法读取 |
| 运行时 URDF 解析 | 用 `tools/urdf2c.py` 离线转换 |

## 安装

**方式一，单头文件。** 从
[Releases](https://github.com/YueWang996/robot-dynamics-c/releases/latest)
下载 `robot_dynamics.h`，放进工程。在其中一个 `.c` 文件里：

```c
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

其他文件直接 `#include "robot_dynamics.h"`。

**方式二，源码树 + CMake。**

```bash
git clone https://github.com/YueWang996/robot-dynamics-c
```

```cmake
add_subdirectory(RobotDynamics)
target_link_libraries(my_firmware PRIVATE robot_dynamics)
```

## 文档

| | |
|---|---|
| @subpage quickstart "快速开始" | 转换模型，写出第一个可运行的程序 |
| @subpage conventions "数据格式" | 向量长度、四元数顺序、参考系、单位 |
| @subpage api "功能与用法" | 每个功能的函数、参数、示例 |
| @subpage contacts "接触与闭链" | 脚踩地面、闭链机构、已知外力 |
| @subpage configuration "编译选项" | 全部编译宏与 CMake 选项 |
| @subpage performance "性能数据" | 五块板子的实测结果 |

[API 参考](files.html)由头文件生成，为英文。

## 许可

Apache License 2.0，允许商用与闭源修改。分发时需保留许可证和 NOTICE 文件，
并说明修改内容。含明确的专利授权条款。

## 相关项目

- [bard](https://github.com/YueWang996/bard-pytorch-dynamics) — PyTorch 实现，
  `q` 排布和空间代数约定与本库一致，向量可直接互传
- [SPARC](https://github.com/YueWang996/sparc) — 本库的初始应用场景，
  四足机器人的三自由度矢状面脊柱单元
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — 正确性验证的参考实现
