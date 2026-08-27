# 接触与闭链 {#contacts}

本库对"机器人与外界接触"提供两套接口，区别在于已知量是力还是运动。

| 已知条件 | 使用 |
|---|---|
| 力已知（测力计读数、负载重量、推进器推力） | rd_rnea_ext()、rd_aba_ext() |
| 运动已知（脚踩在地上，接触点不允许加速） | rd_constrained_dynamics() |
| 两者都有（背着负载站立） | rd_constrained_dynamics_ext() |

选错会得到看似合理但错误的结果，不会报错。

## 已知外力

### 数据格式

`f_ext` 是长度 `6 * n_nodes` 的数组，每个连杆一个空间力，表达在**该连杆自身坐标系**：

```
f_ext[6*i + 0..2]   力，N
f_ext[6*i + 3..5]   力矩，N·m
```

没有受力的连杆填零。整个数组传 `NULL` 表示无外力。

### 示例

```c
static rd_real_t f_ext[6 * RD_MAX_LINKS];
memset(f_ext, 0, sizeof(f_ext));

/* 已知世界系下作用在连杆 i 原点的力 w[3]，转到连杆坐标系 */
rd_real_t T[16];
rd_forward_kinematics(&chain, &state, i, T);
/* T 的旋转部分是列主序 3x3，转置即为世界系到体坐标系 */
f_ext[6*i + 0] = T[0]*w[0] + T[1]*w[1] + T[2]*w[2];
f_ext[6*i + 1] = T[4]*w[0] + T[5]*w[1] + T[6]*w[2];
f_ext[6*i + 2] = T[8]*w[0] + T[9]*w[1] + T[10]*w[2];

rd_rnea_ext(&chain, &state, qdd, NULL, f_ext, tau);
```

### 说明

外力在 O(n) 递推内部施加，一个接触点只增加几次加法。常见的 `tau += Jᵀ·f` 写法需要
为每个接触点计算一次雅可比并做转置乘法，开销大得多。

脚通常是固定连杆，rd_chain_build() 会把固定连杆折进最近的可动祖先。往固定连杆上加力
仍然有效：数组按连杆编号索引，折叠时外力会随惯量一起传递到可动祖先。

## 约束：接触与闭链

### 适用场景

**接触。** 踩在地面上的脚。接触力未知，已知的是接触点不允许加速，力由求解得出。

**闭链。** URDF 只能描述树结构，无法表达"这两根连杆连在一起"。做法与 Pinocchio
一致：模型保留树结构，被剪开的两个 frame 用一条约束绑定。

两者用同一个数据结构。

### 约束类型

```c
typedef enum {
    RD_CONSTRAINT_POINT = 0,   /* 3 行：两个 frame 的原点重合，可相对转动 */
    RD_CONSTRAINT_FULL  = 1    /* 6 行：原点重合且不可相对转动，相当于焊接 */
} rd_constraint_type_t;

typedef struct {
    rd_idx_t             frame_a;   /* 约束写在这个 frame 的原点处 */
    rd_idx_t             frame_b;   /* 另一端。RD_ANCHOR_WORLD 表示接地 */
    rd_constraint_type_t type;
} rd_constraint_t;
```

@warning `frame_b` 是索引，接地用 `RD_ANCHOR_WORLD`，值为 `-1`。
写成 `RD_FRAME_WORLD` 可以编译通过，但那是值为 `0` 的枚举，当索引用表示 0 号连杆，
也就是基座，结果是把脚约束到了机身上。

### 函数

```c
rd_int_t    rd_constrained_dynamics_work(const rd_chain_t* chain,
                                         const rd_constraint_t* cons, rd_int_t n_cons);

rd_status_t rd_constrained_dynamics(const rd_chain_t* chain, const rd_state_t* state,
                                    const rd_real_t* tau, const rd_real_t* gravity,
                                    const rd_constraint_t* cons, rd_int_t n_cons,
                                    rd_real_t* work,
                                    rd_real_t* qdd_out, rd_real_t* lambda_out);
```

| 参数 | 说明 |
|---|---|
| `cons` | 约束数组 |
| `n_cons` | 约束条数 |
| `work` | 临时数组，长度由 rd_constrained_dynamics_work() 给出 |
| `qdd_out` | 输出关节加速度，长度 `nv` |
| `lambda_out` | 输出接触力，长度为总行数，用 rd_constraint_rows() 计算 |

`lambda_out` 中每条约束的分量写在 `frame_a` 原点处、坐标轴与世界系对齐的参考系里。

### 示例

```c
rd_constraint_t con[2] = {
    { fl_foot, RD_ANCHOR_WORLD, RD_CONSTRAINT_POINT },   /* 左前脚踩地，3 行 */
    { link_a,  link_b,          RD_CONSTRAINT_FULL  },   /* 五连杆闭合，6 行 */
};

rd_int_t rows = rd_constraint_rows(con, 2);              /* = 9 */
rd_int_t nw   = rd_constrained_dynamics_work(&chain, con, 2);

static rd_real_t work[WORK_MAX];
static rd_real_t lambda[9];

rd_constrained_dynamics(&chain, &state, tau, NULL, con, 2, work, qdd, lambda);
```

约束是传入的参数，不属于 chain。足式机器人抬脚时传一个更短的数组即可，
不需要重建 chain，也不产生内存分配。

### 求解的方程

```
[ M   Jᵀ ] [  qdd  ]   [ tau - h ]
[ J   0  ] [ -λ    ] = [ -gamma  ]
```

`M` 只分解一次，每条约束行做一次回代。

### 单独取约束雅可比和偏置项

```c
rd_int_t    rd_constraint_jacobian_work(const rd_chain_t* chain);   /* = 6*nv */

rd_status_t rd_constraint_jacobian(const rd_chain_t* chain, const rd_state_t* state,
                                   const rd_constraint_t* cons, rd_int_t n_cons,
                                   rd_real_t* work, rd_real_t* J_out);

rd_status_t rd_constraint_bias(const rd_chain_t* chain, const rd_state_t* state,
                               const rd_constraint_t* cons, rd_int_t n_cons,
                               rd_real_t* gamma_out);
```

`J_out` 是 `rows × nv` 行主序，`gamma_out` 长度 `rows`。

需要自行组装 KKT 方程，或者在上层实现带摩擦锥的接触求解器时使用。

## 摩擦锥

rd_constrained_dynamics() 解的是**等式**约束，不知道地面只能推不能拉，
也不知道切向力有上限。为满足约束，它会给出把脚往下拉的法向力，
以及任意大的切向力。

以下检查由调用方完成：

```c
/* 对每个点接触，lambda 的三个分量在世界对齐坐标系下 */
rd_real_t fx = lambda[3*k+0], fy = lambda[3*k+1], fz = lambda[3*k+2];

if (fz < 0)                              /* 法向力为负，脚在被往下拉 */
    /* 该脚实际已离地，从约束集中去掉后重新求解 */;

if (fx*fx + fy*fy > mu*mu*fz*fz)         /* 超出摩擦锥 */
    /* 切向力送不出去，需要重新分配或降低期望加速度 */;
```

强制满足这两条需要迭代（去掉不满足的脚重新求解），迭代策略属于控制决策，
本库不做。

## 自检方法

把求解得到的接触力反代回 rd_rnea_ext()，应复现出输入的力矩：

```c
rd_constrained_dynamics(&chain, &state, tau, NULL, con, n, work, qdd, lambda);

/* 把 lambda 转成 f_ext 后 */
rd_rnea_ext(&chain, &state, qdd, NULL, f_ext, tau_check);
/* tau_check 应等于 tau */
```

`examples/go2_contact/` 中这个闭环误差为 1.5e-05。

这个检查不需要参考实现，可以在目标板上直接跑，能发现符号错误和参考系错误。

## 完整示例

`examples/go2_contact/` 是一个四足机器人的完整例子：

```bash
cd examples/go2_contact && make && ./go2_contact
```

覆盖四脚站立、力分配、两脚支撑、摩擦锥扫描、闭环自检。其中几个值得注意的结果：

- 四脚踩住但关节力矩为零时，机身仍以接近 1 g 下落。约束保证脚不动，
  站立需要力矩，接触求解只给出地面反力（158 N 体重中的 10 N）。
- 按体重平均分配足底力后，`tau[0..5]` 不为零，说明平均分配没有满足基座平衡。
  求解这个残差属于力分配问题，一般是一个小规模 QP。
- 支撑集变化时力矩必须重算。四脚的力矩用在两脚支撑上，会有一半体重无人承担。

## 计算开销

STM32L413 @ 80 MHz，Go2，含 rd_update_kinematics()：

| | 耗时 |
|---|---|
| 无接触，rd_forward_dynamics() 用 CRBA | 435 µs |
| 两个点接触 | 1039 µs |

约 2.6 倍。增量来自约束雅可比、偏置项，以及每个约束行一次 Cholesky 回代
（两个点接触共 6 行）。质量矩阵两种情况下都只分解一次。
