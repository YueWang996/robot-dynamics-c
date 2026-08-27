# API 一览 {#api}

按"你要算什么"分组。下面每个名字都链进[参考手册](files.html)，那部分由头文件生成。

## 启动时做的事

只做一次。rd_chain_build() 是库里唯一会分配内存的函数。

| | |
|---|---|
| rd_chain_build() | 把 rd_model_t 变成算法能走的形式 |
| rd_chain_free() | 把它分配的还回去 |
| rd_chain_find_frame() | 按连杆名拿 frame 索引 |
| rd_chain_get_nv() | `nv`：`qd`、`qdd`、`tau` 的长度 |
| rd_chain_set_armature() | 设某个关节的折算转子惯量 |
| rd_state_buffer_size() | n 连杆模型需要多少字节工作区 |
| rd_state_init() | 把 rd_state_t 指到那块缓冲区上 |

## 每周期一次

| | |
|---|---|
| rd_update_kinematics() | 变换和速度，后面所有算法都读它 |

先调它，然后想跑多少算法就跑多少，都对着同一个 state。`q` 变了却没调它，
你会安静地拿到上一周期的答案。

## 运动学

| | |
|---|---|
| rd_forward_kinematics() | 从缓存里取某 frame 在世界系的 4×4 位姿 |
| rd_fk_frame() | 同样的位姿，不用 state，沿根到该 frame 的路径走一遍 |
| rd_jacobian() | 几何雅可比，`6 × nv` 行主序 |
| rd_spatial_velocity() | 某 frame 的空间速度 |
| rd_spatial_acceleration() | 给定 `qdd` 时它的空间加速度 |

rd_fk_frame() 收的是 `q_base` 和 `q_joints` 而不是 state，所以它能回答"脚在这个位形下
会在哪"这类问题，同时不动本周期的缓存。它的代价是走一条路径；
rd_forward_kinematics() 几乎不要钱，但要求缓存是当前周期的。

## 逆动力学

| | |
|---|---|
| rd_rnea() | `tau = M(q) qdd + C(q,qd) qd + g(q)` |
| rd_rnea_ext() | 同上，外加任意连杆上一个已知外力 |
| rd_gravity() | 单独的 `g(q)` |
| rd_nonlinear_terms() | `C(q,qd) qd + g(q)`，即 `M qdd + h` 里的 `h` |
| rd_coriolis() | `C(q,qd) qd`，去掉重力 |

## 质量矩阵与正动力学

| | |
|---|---|
| rd_crba() | `M(q)`，`nv × nv`，两个三角都填满 |
| rd_aba() | 由 `tau` 求 `qdd`，O(n)，不构造质量矩阵 |
| rd_aba_ext() | 同上，带外力 |
| rd_forward_dynamics() | 两种方法都行，用参数选 |
| rd_forward_dynamics_work() | 该方法要多大的临时空间 |

哪个更快是机器人的性质，而且不是自由度数决定的。ABA 的关节体惯量同余变换有一条快路径，
条件是关节原点不带旋转，多数 URDF 满足：Go2 十二个关节全都走这条路。xarm7 的原点是
四分之一转，一个都走不上，所以 xarm7 是 CRBA 唯一赢的模型。浮动基座又往反方向推，
因为它的六个自由度是所有关节的祖先，质量矩阵没有稀疏性可供分解利用。
实测幅度见 @ref performance "性能"。

rd_aba() 和 rd_aba_ext() 只在 `RD_ENABLE_ABA` 为 1 时编译，这是默认值。
关掉能换来什么见 @ref configuration "编译配置"。

## 线性代数

| | |
|---|---|
| rd_cholesky_factor() | 对称正定矩阵的 `L Lᵀ` 分解，原地 |
| rd_cholesky_solve() | 用这份分解解一次 |

公开出来，是因为操作空间惯量、任务空间控制器和带约束的求解都要一个分解好的质量矩阵，
不该让它们各自再带一份。一次分解可以服务任意多个右端项。

## 约束与接触

| | |
|---|---|
| rd_constraint_rows() | 约束集贡献多少行：点约束 3 行，全约束 6 行 |
| rd_constraint_jacobian_work() | 下一个函数要的临时空间：`6*nv` |
| rd_constraint_jacobian() | 约束雅可比 `J` |
| rd_constraint_bias() | 与之配套的 `gamma` |
| rd_constrained_dynamics_work() | 完整求解要的临时空间 |
| rd_constrained_dynamics() | 一次给出 `qdd` 和接触力 `lambda` |
| rd_constrained_dynamics_ext() | 同上，再加一个已知外力 |

这些看 @ref contacts "接触与闭链"，包括落地的脚该用哪一个。

## 类型

| | |
|---|---|
| rd_model_t | 作为数据的机器人。`tools/urdf2c.py` 生成 |
| rd_chain_t | 该模型预处理成可遍历的形式 |
| rd_state_t | 一个周期的缓存，外加所有算法的临时空间 |
| rd_constraint_t | 两个必须保持相对静止的 frame |
| rd_status_t | 每个函数的返回值。`RD_OK` 是 0 |
| rd_frame_t | 结果表达在哪个系 |
| rd_joint_type_t、rd_axis_t | 关节能做什么、绕哪根轴 |
| rd_real_t | `float` 或 `double`，由 RD_USE_SINGLE_PRECISION 决定 |

## 谁需要临时缓冲区

多数函数完全在 rd_state_t 里干活。有四个要额外的数组，每个旁边都配了一个问大小的函数，
不需要记公式：

| 函数 | 问它 |
|---|---|
| rd_forward_dynamics() | rd_forward_dynamics_work() |
| rd_constraint_jacobian() | rd_constraint_jacobian_work() |
| rd_constrained_dynamics() | rd_constrained_dynamics_work() |
| rd_constrained_dynamics_ext() | rd_constrained_dynamics_work() |

这几个问大小的函数很便宜，而且只依赖 chain 和约束集，所以控制环在启动时问一次，
然后静态声明数组。
