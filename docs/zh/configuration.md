# 编译配置 {#configuration}

所有选项都是预处理宏，默认值写在 `rd_config.h` 里。用单头文件的人在编译命令行上加
`-D`，用 CMake 的人大部分能拿到一个现成的 `option()`。

## 选项表

| 宏 | CMake 选项 | 默认 | 作用 |
|---|---|---|---|
| RD_USE_SINGLE_PRECISION | `RD_SINGLE_PRECISION` | 1 | rd_real_t 用 `float`；设 0 用 `double` |
| RD_FAST_TRIG | `RD_FAST_TRIG` | 1 | 用多项式算 `sin`/`cos`，替掉 libm 的 |
| RD_ENABLE_ABA | `RD_ENABLE_ABA` | 1 | 编译 ABA 正动力学 |
| RD_MATH_BACKEND | 无 | 未设 | 指向你自己的头文件，里面定义 `RD_SINCOS` 和/或 `RD_SQRT` |
| RD_MAX_LINKS | 无 | 16 | 一个 rd_model_t 最多装多少连杆 |
| RD_MAX_JOINTS | 无 | 12 | 最多装多少驱动关节 |
| RD_USE_CMSIS_DSP | `RD_CMSIS_DSP` | 0 | `sqrt` 改用 CMSIS-DSP 的，需要配 `RD_CMSIS_DSP_INCLUDE_DIR` |
| RD_USE_STATIC_ALLOC | `RD_STATIC_ALLOC` | 0 | 让内存分配宏返回 NULL |
| `RD_DEBUG` | `RD_ENABLE_DEBUG` | 0 | 打开断言和日志 |
| 无 | `RD_OPTIMIZE_SIZE` | OFF | 用 `-Os` 代替 `-O3 -ffast-math` |

@warning RD_USE_SINGLE_PRECISION 和 RD_ENABLE_ABA 会改变结构体的内存布局。整个程序
里每个编译单元都得用相同的值，否则分配 state 缓冲区的那个 `.c` 和读它的那个 `.c`
会对字段偏移量有不同理解，结果是读到别的字段上去，而且不会崩，只是数字不对。CMake
把这两个标成 `PUBLIC` 就是为了防这件事。

## 单精度还是双精度

Cortex-M4F 和 M33 的硬件 FPU 只有单精度。用 `double` 编译，所有浮点运算会退回软件
实现，而这个库瞄准的正是这类芯片。

单精度和 Pinocchio 在所有测试模型上相差 1.9e-06。这个数比喂进去的编码器分辨率和惯量
参数本身的误差都小，所以默认是它。双精度能到 5.5e-15，适合在主机上核对结果时用。

## 快速三角函数

关节角要转成旋转矩阵，每个转动关节每周期都得算一次 `sin` 和 `cos`。RD_FAST_TRIG
把 libm 的实现换成一对多项式：每对 `(sin, cos)` 169 个周期，libm 是 521 个。这是在
STM32G474 @ 170 MHz 上、在 rd_update_kinematics() 内部实测的。

精度方面，在 [-π, π] 上对照双精度，多项式最坏误差 6.6e-08，libm 是 5.9e-08。
实际用起来是一回事，两者以相同容差通过 Pinocchio 比对。

在 Go2 上这个开关能砍掉 rd_update_kinematics() 45% 的时间，是这一页上单个收益最大的
选项。

## 关掉 ABA 能省什么

ABA 是这里唯一需要为每个节点单独存中间量的算法：一个关节体惯量、一个速度积加速度、
外推时用的 U/D/u 三元组。rd_state_t 的大小就是被它撑起来的。

| | 每根连杆占用 |
|---|---|
| `RD_ENABLE_ABA=1` | 70 个浮点数 |
| `RD_ENABLE_ABA=0` | 45 个浮点数 |

40 连杆的模型在 float32 下，是 11,264 字节和 7,264 字节的差别。

如果你的正动力学已经定下来走 `rd_forward_dynamics(RD_FD_CRBA)`，或者压根只需要逆
动力学，就把它设成 0。这时 rd_aba() 和 rd_aba_ext() 不再声明，rd_forward_dynamics()
遇到 `RD_FD_ABA` 会返回 `RD_ERR_INVALID_INDEX`。

flash 两边差不了多少，因为一个从不调用 ABA 的程序，链接时 `--gc-sections` 本来就会
把它丢掉。省的是 RAM。

## 模型能装多大

RD_MAX_LINKS 和 RD_MAX_JOINTS 是 rd_model_t 内部数组的长度。那个结构体把数组内联在
自己身上，所以这两个宏直接决定程序里每个模型对象占多少空间，跟机器人有没有装满无关。

| 机器人 | 连杆 | 关节 |
|---|---|---|
| xarm7 | 8 | 7 |
| spine | 10 | 3 |
| Go2 | 31 | 12 |
| G1 | 40 | 29 |

固定连杆也算连杆。Go2 那 31 个里大部分是脚、传感器支架、惯性系这类不能动的东西，
rd_chain_build() 会把它们折进最近的可动祖先，所以动力学算的时候根本不走它们。

## 换成你芯片的硬件三角函数

有些 MCU 带数学协处理器，比如 STM32G4 和 H7 上的 CORDIC。RD_MATH_BACKEND 就是接它们
的口子。

把 RD_MATH_BACKEND 定义成你自己那个头文件的名字，`rd_math.h` 会在定义任何东西之前
先 include 它。你的头文件里定义了下面哪个宏，哪个就顶掉库自带的实现：

| | |
|---|---|
| `RD_SINCOS(x, sp, cp)` | 算 `x` 的正弦和余弦，分别写进 `*(sp)` 和 `*(cp)` |
| `RD_SQRT(x)` | 算 `x` 的平方根，写成一个表达式 |

只定义其中一个也可以，另一个继续用库自带的。芯片只有平方根加速器就只定义 `RD_SQRT`。

做成宏而不用函数指针，是因为 rd_sincos() 在 rd_update_kinematics() 的循环里每个转动
关节调一次，在 Go2 上占该函数 45% 的时间。在这种位置放一次间接调用，光是调用开销就
超过多数加速器省下来的时间，而且编译器没法把调用方的中间值留在寄存器里跨过它。

库里不带任何现成的 backend。`robot_dynamics.h` 保持单个文件、背后没有厂商头文件。
仓库的 `examples/backends/` 里有一个 STM32G4 CORDIC 的完整实现，可以照抄。

### 三种接法

这个口子是宏，所以库以源码树形式还是单文件形式到手都一样用。合并成单文件之后
`#include RD_MATH_BACKEND` 这行仍然有效，因为它写的是宏名，合并脚本不会把它当成
需要内联的本地头文件。

只换一个运算，连文件都不用建：

```c
#define RD_SINCOS(x, sp, cp)  my_sincos((x), (sp), (cp))
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

给一整个文件，CORDIC 例子就是这样：

```c
#define RD_MATH_BACKEND  "my_cordic.h"
#define RD_IMPLEMENTATION
#include "robot_dynamics.h"
```

从构建系统给，源码一个字都不用改。那串引号是头文件名活着到达预处理器所必需的：

```make
CFLAGS += '-DRD_MATH_BACKEND="my_cordic.h"'
```

@warning 只有一条规则：你选的这个设置必须传到定义了 `RD_IMPLEMENTATION` 的那个编译
单元，因为库本身是在那里编译的。从构建系统全局设置，就再也不用想这件事。
另一个编译单元 include 头文件时没带这个设置不会弄坏什么，因为没有任何结构体布局依赖
backend；但从那个文件里直接调 rd_sincos() 拿到的是库自带的实现，两边在末几位上会有
出入。

### 自己写一个要注意什么

有两件事做错了都是安静地损失精度，测速的时候还看不出来。

**参数规约要分两步做。** 先除以 π 再取整，余下的部分只带着 `x/π` 这个量级的数字的
低位。`x = 200` 时那是 3.8e-06 的 ulp，最后答案会错 1.4e-05。正确做法是把 `k·π`
拆成高位和低位两个数分别减掉，这样有效位才保得住。

**先验精度再测速度。** 在 `benchmark/stm32g4` 下跑 `make TRIG=1`，它会在一整圈
以及 ±200 弧度的范围内，用双精度拿你的 rd_sincos() 和 libm 对扫一遍，打印最坏误差。
一个又快又错的 backend 应该栽在这里，别让它栽在机器人上。

还有一个不那么直观的连带影响。sin/cos 变粗糙会改变 rd_chain_build() 算出来的关节
偏置矩阵，而库正是靠比对这个矩阵来判断哪些关节能走 CRBA 和 ABA 的轴对齐快速路径。
判断失误一次，代价是 Go2 的 ABA 慢了 14%。后来把判断改成直接看模型里的原始数据，
不看算出来的矩阵，才解决。
