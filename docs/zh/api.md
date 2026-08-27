# API 一览 {#api}

按"你想算什么"分组。每个函数名都链到[参考手册](files.html)，那部分由头文件生成。

## 启动时做的事

这几个只做一次。rd_chain_build() 是库里唯一向堆申请内存的函数。

| | |
|---|---|
| rd_chain_build() | 把 rd_model_t 预处理成算法能高效遍历的形式 |
| rd_chain_free() | 释放上面申请的内存 |
| rd_chain_find_frame() | 按连杆名查 frame 索引 |
| rd_chain_get_nv() | 拿到 `nv`，也就是 `qd`、`qdd`、`tau` 的长度 |
| rd_chain_set_armature() | 给某个关节填折算转子惯量 |
| rd_state_buffer_size() | 问 n 个连杆的模型需要多少字节工作区 |
| rd_state_init() | 把 rd_state_t 指到你分配的那块缓冲区上 |

预处理都做了些什么：排好拓扑序、把每根连杆的惯量平移到自己的原点、把固定连杆折进
最近的可动祖先、判断哪些关节的偏置满足快速算法的条件。这些结果每个周期都要用，
所以算一次存下来。

## 每个周期做一次

| | |
|---|---|
| rd_update_kinematics() | 走一遍树，算出每根连杆的位姿和速度 |

先调它，然后同一个 `state` 上想跑几个算法就跑几个。`q` 变了却忘了调它，你会拿到
上一个周期的结果，而且没有任何提示。

## 运动学

| | |
|---|---|
| rd_forward_kinematics() | 某个 frame 在世界系的 4×4 位姿，从缓存里取 |
| rd_fk_frame() | 同样是位姿，但不用 `state`，自己沿根到该 frame 的路径走一遍 |
| rd_jacobian() | 几何雅可比，`6 × nv` 行主序 |
| rd_spatial_velocity() | 某个 frame 的空间速度 |
| rd_spatial_acceleration() | 给定 `qdd` 时它的空间加速度 |

两个正运动学函数的区别在于用途。rd_forward_kinematics() 读本周期的缓存，几乎不花
时间，前提是你已经调过 rd_update_kinematics()。rd_fk_frame() 收的是位形本身，
所以它可以回答"假如关节转到这个角度，脚会落在哪"这类假设性问题，同时不动本周期
的缓存——做碰撞预判或者简单的逆运动学迭代时会用到。代价是它要现走一遍路径。

## 逆动力学：知道运动，求力矩

| | |
|---|---|
| rd_rnea() | `tau = M(q) qdd + C(q,qd) qd + g(q)`，完整的逆动力学 |
| rd_rnea_ext() | 同上，但可以在任意连杆上加一个已知的外力 |
| rd_gravity() | 只要重力项 `g(q)`，做重力补偿用 |
| rd_nonlinear_terms() | `C(q,qd) qd + g(q)`，也就是 `M qdd + h` 里的那个 `h` |
| rd_coriolis() | `C(q,qd) qd`，把重力去掉的部分 |

## 质量矩阵与正动力学：知道力矩，求运动

| | |
|---|---|
| rd_crba() | `M(q)`，`nv × nv`，上下三角都填满 |
| rd_aba() | 直接由 `tau` 算 `qdd`，O(n)，中间不构造质量矩阵 |
| rd_aba_ext() | 同上，带外力 |
| rd_forward_dynamics() | 上面两个方法都能选，用一个参数指定 |
| rd_forward_dynamics_work() | 问选定的方法要多大临时空间 |

**该选哪个方法。** 两个算出来的 `qdd` 一样，快慢取决于机器人本身，而且自由度数不是
决定因素。

ABA 内部要做一步叫"关节体惯量同余变换"的矩阵运算。当一个关节的安装偏置不带旋转时，
这步可以走一条便宜得多的路径，而大多数 URDF 都满足这个条件——Go2 的十二个关节全都
满足。xarm7 的关节偏置是四分之一转，一个都不满足，ABA 只能走通用路径，所以 xarm7
是 CRBA 唯一赢的模型。

浮动基座往反方向推。CRBA 要解一个 `nv × nv` 的方程组，代价按 nv³ 长；而基座那六个
自由度是所有关节的祖先，会把质量矩阵填满，没有稀疏性可以利用。所以机器人越大、
越是浮动基座，ABA 越有优势。

实测数字在 @ref performance "性能" 里。

rd_aba() 和 rd_aba_ext() 只在 `RD_ENABLE_ABA` 为 1 时才编译，默认就是 1。
关掉它能省下什么见 @ref configuration "编译配置"。

## 线性代数

| | |
|---|---|
| rd_cholesky_factor() | 对称正定矩阵的 `L Lᵀ` 分解，原地进行 |
| rd_cholesky_solve() | 用分解结果解一次 |

这两个暴露出来，是因为操作空间惯量、任务空间控制器、带约束的求解都需要一个分解好的
质量矩阵，不该让每个用到的人各自再写一份。一次分解可以反复解不同的右端项，
`J M⁻¹ Jᵀ` 就是这么算的：分解一次，解六次。

## 接触与闭链

| | |
|---|---|
| rd_constraint_rows() | 这组约束一共贡献多少行：点约束 3 行，全约束 6 行 |
| rd_constraint_jacobian_work() | 下面那个函数要的临时空间，`6*nv` |
| rd_constraint_jacobian() | 约束雅可比 `J` |
| rd_constraint_bias() | 配套的偏置项 `gamma` |
| rd_constrained_dynamics_work() | 完整求解要的临时空间 |
| rd_constrained_dynamics() | 一次解出 `qdd` 和接触力 `lambda` |
| rd_constrained_dynamics_ext() | 同上，再叠加一个已知外力 |

前四个是给想自己组装 KKT 方程或者在上面写接触求解器的人用的。只想要结果就用后两个。
细节，包括"踩住的脚该用哪个函数"，见 @ref contacts "接触与闭链"。

## 类型

| | |
|---|---|
| rd_model_t | 机器人的静态描述，`tools/urdf2c.py` 生成 |
| rd_chain_t | 预处理之后的形式，算法遍历的是它 |
| rd_state_t | 一个周期的缓存，外加所有算法共用的临时空间 |
| rd_constraint_t | 一条约束：两个必须保持相对静止的 frame |
| rd_status_t | 所有函数的返回类型，`RD_OK` 是 0 |
| rd_frame_t | 结果表达在哪个参考系里 |
| rd_joint_type_t、rd_axis_t | 关节能做什么运动、绕哪根轴 |
| rd_real_t | `float` 或 `double`，由 RD_USE_SINGLE_PRECISION 决定 |

## 哪些函数需要额外的临时数组

多数函数在 rd_state_t 里就干完了。有四个需要你另外给一块数组，每个旁边都配了一个
问大小的函数，不用记公式：

| 函数 | 大小问它 |
|---|---|
| rd_forward_dynamics() | rd_forward_dynamics_work() |
| rd_constraint_jacobian() | rd_constraint_jacobian_work() |
| rd_constrained_dynamics() | rd_constrained_dynamics_work() |
| rd_constrained_dynamics_ext() | rd_constrained_dynamics_work() |

这几个问大小的函数很便宜，而且结果只跟 chain 和约束集有关，跟当前位形无关。
所以控制环在启动时问一次，按最大值静态声明数组，之后就不用管了。
