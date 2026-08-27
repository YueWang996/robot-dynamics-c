# 编译配置 {#configuration}

每个选项都是一个预处理宏，默认值在 `rd_config.h` 里。用单头文件的人在编译命令行上设，
用 CMake 的人大部分能拿到一个 `option()`。

## 选项表

| 宏 | CMake | 默认 | 作用 |
|---|---|---|---|
| RD_USE_SINGLE_PRECISION | `RD_SINGLE_PRECISION` | 1 | rd_real_t 用 `float`。设 `0` 用 `double` |
| RD_FAST_TRIG | `RD_FAST_TRIG` | 1 | 用多项式 `sin`/`cos` 代替 libm 的 |
| RD_ENABLE_ABA | `RD_ENABLE_ABA` | 1 | 编译关节体算法 |
| RD_MATH_BACKEND | — | 未设 | 你自己的头文件，里面定义 `RD_SINCOS` 和/或 `RD_SQRT` |
| RD_MAX_LINKS | — | 16 | 一个 rd_model_t 能装多少连杆 |
| RD_MAX_JOINTS | — | 12 | 能装多少驱动关节 |
| RD_USE_CMSIS_DSP | `RD_CMSIS_DSP` | 0 | `sqrt` 取自 CMSIS-DSP，要配 `RD_CMSIS_DSP_INCLUDE_DIR` |
| RD_USE_STATIC_ALLOC | `RD_STATIC_ALLOC` | 0 | 让分配宏返回 NULL |
| `RD_DEBUG` | `RD_ENABLE_DEBUG` | 0 | 断言和日志输出 |
| — | `RD_OPTIMIZE_SIZE` | OFF | 用 `-Os` 代替 `-O3 -ffast-math` |

@warning RD_USE_SINGLE_PRECISION 和 RD_ENABLE_ABA 会改变结构体布局。程序里每个编译单元
都必须用相同的值构建，否则分配 state 缓冲区的那个单元和读它的那个单元，对字段在哪里的
理解会不一致。CMake 目标把这两个标成 `PUBLIC` 就是这个原因。

## 精度

Cortex-M4F 和 M33 的 FPU 只有单精度，所以在这个库瞄准的这些芯片上，double 构建的算术
全在软件里跑。

单精度对照 Pinocchio 在所有测试模型上一致到 1.9e-06。这比喂进去的编码器分辨率和惯量
参数都要细，所以它是默认。double 能到 5.5e-15，属于主机上做核对时用的。

## 快速三角函数

RD_FAST_TRIG 用一对多项式换掉 libm 的 `sin` 和 `cos`：每对 `(sin, cos)` 169 个周期，
libm 是 521，在 STM32G474 @ 170 MHz 上于 rd_update_kinematics() 内部实测。精度方面，
在 [-π, π] 上对照 double 最坏 6.6e-08，libm 是 5.9e-08 —— 实际用起来是一回事，
两者都以相同的容差通过 Pinocchio 比对。

在 Go2 上它值 rd_update_kinematics() 的 45%，是这一页上单项收益最大的开关。

## 去掉 ABA

ABA 是这里唯一带自己每节点状态的算法：一个关节体惯量、一个速度积加速度，以及外推需要的
U/D/u 三元组。所以是它决定了 rd_state_t 的大小。

| | 每连杆浮点数 |
|---|---|
| `RD_ENABLE_ABA=1` | 70 |
| `RD_ENABLE_ABA=0` | 45 |

40 连杆的模型在 float32 构建下，是 11,264 字节对 7,264 字节。如果你的正动力学已经定为
`rd_forward_dynamics(RD_FD_CRBA)`，或者只需要逆动力学，就把它设成 0。
这时 rd_aba() 和 rd_aba_ext() 不会被声明，rd_forward_dynamics() 对 `RD_FD_ABA`
返回 `RD_ERR_INVALID_INDEX`。

flash 两边差不多，因为 `--gc-sections` 在一个从不调用 ABA 的程序里本来就会把它丢掉。
省的是 RAM。

## 模型规模

RD_MAX_LINKS 和 RD_MAX_JOINTS 决定 rd_model_t 内部数组的上界，而那个结构体是把数组
内联在自己身上的，所以它们决定了程序里每个模型对象的大小，跟机器人有没有装满无关。

| 机器人 | 连杆 | 关节 |
|---|---|---|
| xarm7 | 8 | 7 |
| spine | 10 | 3 |
| Go2 | 31 | 12 |
| G1 | 40 | 29 |

固定连杆也算连杆。Go2 那 31 个里大部分是脚、传感器安装座和惯性系，
rd_chain_build() 会把它们折进最近的可动祖先，所以动力学根本不走它们。

## 换掉自带的 sin/cos

RD_MATH_BACKEND 指向*你自己的*头文件。`rd_math.h` 在定义任何东西之前先 include 它，
它定义了下面哪个宏，哪个就取代自带的实现：

| | |
|---|---|
| `RD_SINCOS(x, sp, cp)` | `x` 的正弦和余弦，写进 `*(sp)` 和 `*(cp)` |
| `RD_SQRT(x)` | `x` 的平方根，作为一个表达式 |

如果你的芯片只有其中一样，就只定义那一个，另一个别管。

它们做成宏而不是函数指针是有意的。rd_sincos() 在 rd_update_kinematics() 的循环里每个
转动关节调一次，在 Go2 上占该函数的 45%；在那个位置放一次间接调用，代价比多数加速器省
下来的还多，而且会让编译器无法把调用方的值留在寄存器里跨过它。

库里不带任何这类东西。`robot_dynamics.h` 保持单个文件、背后没有厂商头文件，仓库的
`examples/backends/` 里有一个 STM32G4 CORDIC 的完整例子可以读、可以抄。

### 三种接法

这个钩子是宏，所以库以源码树形式还是单文件形式到手，用法都一样。没有任何东西是提前
编译好的，而 `#include RD_MATH_BACKEND` 这一行能在合并成单文件之后依然有效，
因为它写的是宏名而不是字面量。

只换一个运算，不加文件：

```c
#define RD_SINCOS(x, sp, cp)  my_sincos((x), (sp), (cp))
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

整个文件，CORDIC 例子就是这么做的：

```c
#define RD_MATH_BACKEND  "my_cordic.h"
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

从构建系统给，如果你不想动源码。那串引号是头文件名要活着到达预处理器所必须的：

```make
CFLAGS += '-DRD_MATH_BACKEND="my_cordic.h"'
```

@warning 只有一条规则：你选的这个东西必须到达定义了 `RD_IMPLEMENTATION` 的那个编译
单元，因为库本身是在那里编译的。从构建系统上全局设置，就是不用再去想这件事的办法。
另一个编译单元 include 头文件时没带同样的设置不会弄坏任何东西 —— 没有任何结构体布局
依赖 backend —— 但从那里直接调 rd_sincos() 拿到的是自带的实现，两者在最后几位上会不同。

### 自己写一个

有两件事要做对，做错了都是安静地损失精度。

**分两步做参数规约。** 先除以 π 再取整，留下的余量只带着 `x/π` 这个量级的数字的低位。
在 `x = 200` 时那是 3.8e-06 的 ulp，答案会错 1.4e-05。把 `k·π` 拆成高位和低位两部分
减掉，才能保住这些位。

**先验精度再测速度。** 在 `benchmark/stm32g4` 下 `make TRIG=1` 会在一整圈以及
±200 弧度范围内，用 double 拿 rd_sincos() 和 libm 扫一遍并打印最坏误差。一个又快又错的
backend 会栽在这里，而不是栽在别人的机器人上。

精度在看起来免费的地方也不免费。更粗的 sin/cos 会改变 rd_chain_build() 算出来的关节
偏置，而那是决定哪些关节能在 CRBA 和 ABA 里走轴对齐同余变换的构建期输入。这件事弄错
过一次，代价是 Go2 的 ABA 慢了 14%，后来把分类改成依赖模型本身而不是依赖算出来的变换
才解决。
