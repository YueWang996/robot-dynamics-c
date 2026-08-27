# 编译选项 {#configuration}

所有选项都是预处理宏，默认值在 `rd_config.h`。单头文件用户在编译命令行上定义，
CMake 用户大部分有对应的 `option()`。

## 选项总表

| 宏 | CMake 选项 | 默认 | 作用 |
|---|---|---|---|
| `RD_USE_SINGLE_PRECISION` | `RD_SINGLE_PRECISION` | 1 | rd_real_t 为 `float`，设 0 为 `double` |
| `RD_FAST_TRIG` | `RD_FAST_TRIG` | 1 | 多项式 `sin`/`cos` 替代 libm |
| `RD_ENABLE_ABA` | `RD_ENABLE_ABA` | 1 | 编译关节体算法 |
| `RD_MATH_BACKEND` | — | 未定义 | 指向自定义的数学后端头文件 |
| `RD_MAX_LINKS` | — | 16 | rd_model_t 可容纳的连杆数 |
| `RD_MAX_JOINTS` | — | 12 | 可容纳的驱动关节数 |
| `RD_USE_CMSIS_DSP` | `RD_CMSIS_DSP` | 0 | `sqrt` 使用 CMSIS-DSP |
| `RD_USE_STATIC_ALLOC` | `RD_STATIC_ALLOC` | 0 | 内存分配宏返回 NULL |
| `RD_DEBUG` | `RD_ENABLE_DEBUG` | 0 | 断言与日志 |
| — | `RD_OPTIMIZE_SIZE` | OFF | 用 `-Os` 替代 `-O3 -ffast-math` |

@warning `RD_USE_SINGLE_PRECISION` 和 `RD_ENABLE_ABA` 会改变结构体布局。
程序内所有编译单元必须使用相同的值，否则分配 state 缓冲区的编译单元与读取它的编译
单元对字段偏移的理解不一致，读到的是错误数据且不会崩溃。CMake 目标把这两项标为
`PUBLIC` 即为此。

## RD_USE_SINGLE_PRECISION

| | |
|---|---|
| 默认 | 1（`float`） |
| 影响结构体布局 | 是 |

Cortex-M4F 和 M33 的硬件 FPU 只支持单精度，用 `double` 编译会退回软件浮点。

| 精度 | 与 Pinocchio 的误差 |
|---|---|
| float32 | 1.9e-06 |
| float64 | 5.5e-15 |

float32 的误差小于编码器分辨率和惯量参数本身的不确定度，嵌入式场景用默认值即可。
float64 用于主机上的结果核对。

## RD_FAST_TRIG

| | |
|---|---|
| 默认 | 1（开启） |
| 影响结构体布局 | 否 |

关节角转旋转矩阵时需要 `sin` 和 `cos`，每个转动关节每周期各一次。

| 实现 | 每对 (sin, cos) 周期数 | 在 [-π, π] 上的最大误差 |
|---|---|---|
| 多项式（本选项） | 169 | 6.6e-08 |
| libm | 521 | 5.9e-08 |

在 STM32G474 @ 170 MHz 上于 rd_update_kinematics() 内部实测。两者以相同容差通过
Pinocchio 比对。

Go2 上开启此选项可减少 rd_update_kinematics() 约 45% 的耗时。

## RD_ENABLE_ABA

| | |
|---|---|
| 默认 | 1（编译） |
| 影响结构体布局 | 是 |

ABA 是唯一需要每节点私有中间量的算法（关节体惯量、速度积加速度、U/D/u），
rd_state_t 的大小由它决定。

| | 每连杆占用 | 40 连杆模型（float32） |
|---|---|---|
| `RD_ENABLE_ABA=1` | 70 个 rd_real_t | 11,264 字节 |
| `RD_ENABLE_ABA=0` | 45 个 rd_real_t | 7,264 字节 |

**何时设为 0：** 正动力学固定使用 `rd_forward_dynamics(RD_FD_CRBA)`，或只需要逆动力学。

设为 0 后 rd_aba() 和 rd_aba_ext() 不再声明，rd_forward_dynamics() 对 `RD_FD_ABA`
返回 `RD_ERR_INVALID_INDEX`。

flash 占用变化不大，因为 `--gc-sections` 对未调用的 ABA 本来就会丢弃。节省的是 RAM。

## RD_MAX_LINKS 与 RD_MAX_JOINTS

| | |
|---|---|
| 默认 | 16 / 12 |
| 影响结构体布局 | 是（rd_model_t 的大小） |

rd_model_t 把连杆数组内联在结构体内，这两个宏直接决定每个模型对象的大小。

| 机器人 | 连杆 | 关节 |
|---|---|---|
| xarm7 | 8 | 7 |
| spine | 10 | 3 |
| Go2 | 31 | 12 |
| G1 | 40 | 29 |

固定连杆计入连杆数。Go2 的 31 个连杆中多数是脚、传感器支架和惯性系，
rd_chain_build() 会把它们折进最近的可动祖先，动力学计算不遍历它们。

## RD_MATH_BACKEND

| | |
|---|---|
| 默认 | 未定义 |
| 影响结构体布局 | 否 |

用于让带数学协处理器的 MCU（如 STM32G4/H7 的 CORDIC）接管三角函数或平方根。

### 接口

定义 `RD_MATH_BACKEND` 为你的头文件名，`rd_math.h` 会在定义任何内容前包含它。
你的头文件中定义了以下哪个宏，哪个就替换库自带实现：

| 宏 | 语义 |
|---|---|
| `RD_SINCOS(x, sp, cp)` | 计算 `x` 的正弦和余弦，写入 `*(sp)` 和 `*(cp)` |
| `RD_SQRT(x)` | 返回 `x` 的平方根，作为表达式使用 |

只定义其中一个也可以，另一个继续使用库自带实现。

设计为宏而非函数指针：rd_sincos() 在 rd_update_kinematics() 的内层循环中每个转动关节
调用一次，Go2 上占该函数 45% 的耗时。间接调用的开销超过多数加速器的收益，
且会阻止编译器跨调用保持寄存器分配。

### 三种配置方式

只替换一个运算：

```c
#define RD_SINCOS(x, sp, cp)  my_sincos((x), (sp), (cp))
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

使用完整的后端文件：

```c
#define RD_MATH_BACKEND  "my_cordic.h"
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

从构建系统配置，不修改源码：

```make
CFLAGS += '-DRD_MATH_BACKEND="my_cordic.h"'
```

@warning 配置必须到达定义了 `RD_IMPLEMENTATION` 的那个编译单元，库本身在那里编译。
推荐从构建系统全局设置。其他编译单元缺少该配置不会破坏结构体布局，
但从那里直接调用 rd_sincos() 会得到库自带实现，两者末位有差异。

### 实现要点

**参数规约必须分两步。** 先除以 π 再取整，余项只保留 `x/π` 量级数字的低位。
`x = 200` 时 ulp 为 3.8e-06，最终误差 1.4e-05。正确做法是把 `k·π` 拆成高低两部分
分别相减。

**先验证精度再测速度。** `benchmark/stm32g4` 下运行 `make TRIG=1`，会在一整圈和
±200 弧度范围内用双精度对比 rd_sincos() 与 libm，打印最大误差。

**注意对快速路径判定的影响。** sin/cos 精度下降会改变 rd_chain_build() 计算出的关节
偏置矩阵，而 CRBA 和 ABA 的轴对齐快速路径依据该矩阵判定。曾因此导致 Go2 的 ABA
慢 14%，现已改为依据模型原始数据判定。

库不附带任何后端实现。`examples/backends/` 有一份 STM32G4 CORDIC 的完整实现可供参考。

## RD_USE_CMSIS_DSP

| | |
|---|---|
| 默认 | 0 |

`sqrt` 改用 CMSIS-DSP 实现。需要 CMSIS-DSP 在包含路径中，CMake 下还需设置
`RD_CMSIS_DSP_INCLUDE_DIR`。

新项目建议改用 `RD_MATH_BACKEND`，不引入厂商依赖。

## RD_USE_STATIC_ALLOC

| | |
|---|---|
| 默认 | 0 |

使 `RD_MALLOC`、`RD_CALLOC` 返回 NULL。

控制环本身不分配内存，唯一的例外是 rd_chain_build()，它在启动时分配一次。
开启此选项后 rd_chain_build() 会失败，因此目前无法构建完全无堆的版本。
