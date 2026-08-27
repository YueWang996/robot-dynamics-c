# 功能与用法 {#api}

按功能分节。每节给出函数签名、参数说明和示例。完整的类型定义见
[API 参考](files.html)。

所有函数返回 rd_status_t，`RD_OK` 为 0，错误码为负。

## 初始化

以下三步在启动时各做一次。

### rd_chain_build

```c
rd_status_t rd_chain_build(const rd_model_t* model, rd_chain_t* chain);
```

把模型预处理成算法可遍历的形式：排定拓扑序、把各连杆惯量平移到自身原点、
把固定连杆折进最近的可动祖先、标记可用快速路径的关节。

| 参数 | 说明 |
|---|---|
| `model` | `tools/urdf2c.py` 生成的模型 |
| `chain` | 输出。结构体本身由调用方提供，内部指针由本函数分配 |

| 返回 | 含义 |
|---|---|
| `RD_ERR_ALLOC_FAILED` | 堆内存不足 |
| `RD_ERR_INVALID_SIZE` | 模型超过 `RD_MAX_LINKS` 或 `RD_MAX_JOINTS` |

这是库中唯一调用 `malloc` 的函数。用完调 rd_chain_free() 释放。

### rd_state_buffer_size 与 rd_state_init

```c
size_t      rd_state_buffer_size(rd_int_t n_nodes);
rd_status_t rd_state_init(rd_state_t* state, rd_int_t n_nodes,
                          void* buffer, size_t buffer_size);
```

把一块调用方提供的内存划分给各个算法作为工作区。本函数不分配也不释放内存。

| 参数 | 说明 |
|---|---|
| `n_nodes` | 连杆数，取 `chain.n_nodes` |
| `buffer` | 至少 `rd_state_buffer_size(n_nodes)` 字节，按 rd_real_t 对齐 |
| `buffer_size` | 缓冲区实际字节数，用于校验 |

静态声明用宏 `RD_STATE_BUF_FLOATS(n)`，它给出元素个数：

```c
static rd_real_t buf[RD_STATE_BUF_FLOATS(RD_MAX_LINKS)];
rd_state_t state;
rd_state_init(&state, chain.n_nodes, buf, sizeof(buf));
```

每个连杆占 70 个 rd_real_t（`RD_ENABLE_ABA=0` 时 45 个）。

### rd_chain_find_frame

```c
rd_idx_t rd_chain_find_frame(const rd_chain_t* chain, const char* name);
```

按连杆名查 frame 索引，找不到返回 `-1`。线性扫描，在启动时查好存下来。

模型里所有连杆都能查到，包括固定连杆。

## 每周期更新

### rd_update_kinematics

```c
rd_status_t rd_update_kinematics(const rd_chain_t* chain, rd_state_t* state,
                                 const rd_real_t* q_base,
                                 const rd_real_t* q_joints,
                                 const rd_real_t* qd);
```

计算所有连杆的位姿和空间速度，存入 `state`。**每个控制周期开头调用一次**，
其后的算法读取这份结果。

| 参数 | 说明 |
|---|---|
| `q_base` | 长度 7，`[x y z qw qx qy qz]`。固定基座传 `NULL` |
| `q_joints` | 长度 `nj`。传 `NULL` 表示零位形 |
| `qd` | 长度 `nv`。传 `NULL` 表示静止 |

一个周期内可以调用任意多个算法，它们共用这一次的结果。忘记调用不会报错，
得到的是上一周期的数据。

## 运动学

### rd_forward_kinematics

```c
rd_status_t rd_forward_kinematics(const rd_chain_t* chain, const rd_state_t* state,
                                  rd_idx_t frame_id, rd_real_t T_out[16]);
```

某个 frame 在世界系下的位姿，4×4 齐次变换矩阵，**列主序**。

从 `state` 的缓存组合得到，开销很小，要求本周期已调用过 rd_update_kinematics()。

```c
rd_real_t T[16];
rd_forward_kinematics(&chain, &state, foot_id, T);
rd_real_t px = T[12], py = T[13], pz = T[14];   /* 位置在第四列 */
```

### rd_fk_frame

```c
rd_status_t rd_fk_frame(const rd_chain_t* chain,
                        const rd_real_t* q_base, const rd_real_t* q_joints,
                        rd_idx_t frame_id, rd_real_t T_out[16]);
```

同样返回世界系位姿，但直接接受位形，不使用 `state`。

用于回答假设性问题：给定一组关节角，某个 frame 会在哪里。碰撞预判、逆运动学迭代
会用到。不影响本周期的缓存。

开销是沿根到该 frame 的路径走一遍，比 rd_forward_kinematics() 贵。

### rd_jacobian

```c
rd_status_t rd_jacobian(const rd_chain_t* chain, const rd_state_t* state,
                        rd_idx_t frame_id, rd_frame_t ref_frame,
                        rd_real_t* J_out);
```

几何雅可比，`6 × nv` 行主序。前 3 行线速度，后 3 行角速度。

| 参数 | 说明 |
|---|---|
| `ref_frame` | `RD_FRAME_WORLD` 或 `RD_FRAME_LOCAL`，见 @ref conventions "数据格式" |
| `J_out` | 输出，长度 `6 * nv` |

```c
static rd_real_t J[6 * (6 + RD_MAX_JOINTS)];
rd_jacobian(&chain, &state, foot_id, RD_FRAME_WORLD, J);

/* 第 k 列（第 k 个自由度对该 frame 的贡献） */
rd_real_t col_k[6];
for (int r = 0; r < 6; ++r) col_k[r] = J[r*nv + k];
```

### rd_spatial_velocity 与 rd_spatial_acceleration

```c
rd_status_t rd_spatial_velocity(const rd_chain_t* chain, const rd_state_t* state,
                                rd_idx_t frame_id, rd_frame_t ref_frame,
                                rd_real_t v_out[6]);

rd_status_t rd_spatial_acceleration(const rd_chain_t* chain, const rd_state_t* state,
                                    const rd_real_t* qdd,
                                    rd_idx_t frame_id, rd_frame_t ref_frame,
                                    rd_real_t a_out[6]);
```

某个 frame 的空间速度和空间加速度，`[线量, 角量]`。

加速度需要传入 `qdd`。返回的是空间加速度，转换成点的经典加速度还需要加 ω × ṗ，
见 @ref conventions "数据格式"。

## 逆动力学

已知运动，求所需力矩。

### rd_rnea

```c
rd_status_t rd_rnea(const rd_chain_t* chain, const rd_state_t* state,
                    const rd_real_t* qdd, const rd_real_t* gravity,
                    rd_real_t* tau_out);
```

`tau = M(q)·qdd + C(q,q̇)·q̇ + g(q)`，O(n) 递推，不构造质量矩阵。

| 参数 | 说明 |
|---|---|
| `qdd` | 期望关节加速度，长度 `nv`。传 `NULL` 表示零加速度 |
| `gravity` | 世界系重力，长度 3。传 `NULL` 表示 `{0, 0, -9.81}` |
| `tau_out` | 输出力矩，长度 `nv` |

```c
/* 重力补偿 + 期望加速度 */
rd_rnea(&chain, &state, qdd_desired, NULL, tau);

/* 只要重力和科氏力（qdd = 0） */
rd_rnea(&chain, &state, NULL, NULL, tau);
```

浮动基座时 `tau_out[0..5]` 是作用在基座上的合力旋量，那里没有驱动器。

### rd_rnea_ext

```c
rd_status_t rd_rnea_ext(const rd_chain_t* chain, const rd_state_t* state,
                        const rd_real_t* qdd, const rd_real_t* gravity,
                        const rd_real_t* f_ext, rd_real_t* tau_out);
```

同 rd_rnea()，额外考虑作用在各连杆上的已知外力。

| 参数 | 说明 |
|---|---|
| `f_ext` | 长度 `6 * n_nodes`，每个连杆一个空间力，**该连杆自身坐标系**。传 `NULL` 等价于 rd_rnea() |

外力在 O(n) 递推内部施加，每个接触点只多几次加法。

用法见 @ref contacts "接触与闭链"。

### rd_gravity、rd_nonlinear_terms、rd_coriolis

```c
rd_status_t rd_gravity(const rd_chain_t* chain, const rd_state_t* state,
                       const rd_real_t* gravity, rd_real_t* tau_out);

rd_status_t rd_nonlinear_terms(const rd_chain_t* chain, const rd_state_t* state,
                               const rd_real_t* gravity, rd_real_t* tau_out);

rd_status_t rd_coriolis(const rd_chain_t* chain, const rd_state_t* state,
                        rd_real_t* tau_out);
```

| 函数 | 计算 | 用途 |
|---|---|---|
| rd_gravity() | `g(q)` | 重力补偿 |
| rd_nonlinear_terms() | `C(q,q̇)·q̇ + g(q)`，即 `h` | 组装 `M·qdd + h = tau` |
| rd_coriolis() | `C(q,q̇)·q̇` | 需要单独的科氏项时 |

rd_gravity() 内部把缓存的速度置零后跑 RNEA，所以得到的是纯重力项，
不需要为此重建一个 `q̇ = 0` 的 state。

## 正动力学

已知力矩，求加速度。

### rd_forward_dynamics

```c
rd_int_t    rd_forward_dynamics_work(const rd_chain_t* chain, rd_fd_method_t method);

rd_status_t rd_forward_dynamics(const rd_chain_t* chain, const rd_state_t* state,
                                const rd_real_t* tau, const rd_real_t* gravity,
                                rd_fd_method_t method,
                                rd_real_t* work, rd_real_t* qdd_out);
```

| 参数 | 说明 |
|---|---|
| `tau` | 关节力矩，长度 `nv`。传 `NULL` 表示零力矩 |
| `method` | `RD_FD_ABA` 或 `RD_FD_CRBA` |
| `work` | 临时数组，长度由 rd_forward_dynamics_work() 给出 |
| `qdd_out` | 输出加速度，长度 `nv` |

`work` 的长度：`RD_FD_ABA` 为 0，`RD_FD_CRBA` 为 `nv*nv + nv`。

```c
static rd_real_t work[NV_MAX*NV_MAX + NV_MAX];
rd_forward_dynamics(&chain, &state, tau, NULL, RD_FD_CRBA, work, qdd);
```

**方法选择。** 两种方法结果相同，快慢取决于机器人：

| 情况 | 推荐 |
|---|---|
| 浮动基座，关节偏置不带旋转（多数 URDF） | `RD_FD_ABA` |
| 固定基座机械臂，关节偏置带旋转 | `RD_FD_CRBA` |
| 自由度多（nv > 20）且浮动基座 | `RD_FD_ABA` |
| 同时还需要质量矩阵 | `RD_FD_CRBA`，顺便拿到 `M` |
| RAM 紧张 | `RD_FD_ABA`，不需要 `work` |

实测数据见 @ref performance "性能数据"。差距在 4% 到 47% 之间，建议实测自己的模型。

### rd_aba 与 rd_aba_ext

```c
rd_status_t rd_aba(const rd_chain_t* chain, const rd_state_t* state,
                   const rd_real_t* tau, const rd_real_t* gravity,
                   rd_real_t* qdd_out);

rd_status_t rd_aba_ext(const rd_chain_t* chain, const rd_state_t* state,
                       const rd_real_t* tau, const rd_real_t* gravity,
                       const rd_real_t* f_ext, rd_real_t* qdd_out);
```

直接调用关节体算法，不需要 `work` 数组。`f_ext` 格式同 rd_rnea_ext()。

只在 `RD_ENABLE_ABA` 为 1 时编译，默认为 1。

### rd_crba

```c
rd_status_t rd_crba(const rd_chain_t* chain, const rd_state_t* state,
                    rd_real_t* M_out);
```

质量矩阵 `M(q)`，`nv × nv` 行主序，上下三角都填满。

```c
static rd_real_t M[NV_MAX * NV_MAX];
rd_crba(&chain, &state, M);
```

设过 armature 的关节，其对角元已包含折算转子惯量。

## 线性代数

### rd_cholesky_factor 与 rd_cholesky_solve

```c
rd_status_t rd_cholesky_factor(rd_real_t* A, rd_int_t n, rd_real_t* dinv);

rd_status_t rd_cholesky_solve(const rd_real_t* L, const rd_real_t* dinv,
                              const rd_real_t* b, rd_real_t* x, rd_int_t n);
```

对称正定矩阵的 `L·Lᵀ` 分解与回代。

| 参数 | 说明 |
|---|---|
| `A` | 输入矩阵，行主序。**原地覆盖**为下三角的 `L`，上三角保持不变 |
| `dinv` | 长度 `n` 的输出，存 `1/L_ii`，回代时要用 |
| `b`、`x` | 右端项与解，长度 `n`。两者不能是同一块内存 |

分解只读 `A` 的下三角，所以 rd_crba() 的输出可以直接传入。
`RD_ERR_SINGULAR` 表示矩阵不是正定的。

一次分解可以反复回代不同的右端项，`J·M⁻¹·Jᵀ` 就是这样算的：

```c
rd_crba(&chain, &state, M);
rd_cholesky_factor(M, nv, dinv);            /* 分解一次 */

for (int r = 0; r < 6; ++r)                 /* 每行解一次 */
    rd_cholesky_solve(M, dinv, &J[r*nv], &MinvJt[r*nv], nv);
```

## 减速比

### rd_chain_set_armature

```c
rd_status_t rd_chain_set_armature(rd_chain_t* chain, rd_int_t vidx, rd_real_t value);
```

设置某个关节的折算转子惯量，值为 `n²·I_rotor`，`n` 是减速比。

| 参数 | 说明 |
|---|---|
| `vidx` | 速度索引，即该关节在 `qd`/`qdd`/`tau` 中的位置 |
| `value` | 与 `M` 对角元同单位，转动关节为 kg·m² |

URDF 没有这个字段，转换出来的模型该项为零，需要在 rd_chain_build() 之后设置：

```c
rd_chain_build(&my_robot, &chain);
for (int j = 0; j < chain.n_joints; ++j)
    rd_chain_set_armature(&chain, 6 + j, gear_ratio*gear_ratio*rotor_inertia);
```

设置后 rd_crba() 会把它加到 `M` 的对角线上，rd_rnea() 加到 `tau` 上，
rd_aba() 加到关节体惯量上，三者保持一致。

高减速比驱动（舵机、行星减速电机）的折算惯量常常大于所驱动的连杆本身，
不设置会导致算出的力矩偏小。

## 释放

### rd_chain_free

```c
void rd_chain_free(rd_chain_t* chain);
```

释放 rd_chain_build() 分配的内存。rd_chain_t 结构体本身不释放，可以重新 build。

调用后与该 chain 关联的 rd_state_t 数据失效，重新使用前需要再次调用
rd_update_kinematics()。
