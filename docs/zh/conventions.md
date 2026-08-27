# 数据格式 {#conventions}

本页列出所有向量的排布方式和参考系定义。这些约定与
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) 和
[bard](https://github.com/YueWang996/bard-pytorch-dynamics) 一致，向量可以直接互传。

@warning 本页所有项目弄错后程序都不会报错，函数照常返回 `RD_OK`。请在调试初期逐条核对。

## 维数

| 符号 | 含义 | 值 |
|---|---|---|
| `nj` | 驱动关节数 | `chain.n_joints` |
| `nv` | 速度空间维数 | 浮动基座 `6 + nj`，固定基座 `nj`。用 rd_chain_get_nv() 取 |
| `n_nodes` | 连杆总数，含固定连杆 | `chain.n_nodes` |

浮动基座下位形维数比速度维数大 1，因为姿态用四元数表示，4 个数对应 3 个自由度。
库把位形拆成两个参数传，调用方不需要处理这个差异。

## 向量排布

| 变量 | 长度 | 排布 |
|---|---|---|
| `q_base` | 7 | `[x, y, z, qw, qx, qy, qz]` |
| `q_joints` | `nj` | 每个驱动关节一个，按关节序 |
| `qd` | `nv` | 浮动基座时前 6 个是基座速度旋量，其后是关节速度 |
| `qdd` | `nv` | 同 `qd` |
| `tau` | `nv` | 同 `qd`。前 6 个是作用在基座上的合力旋量 |
| `M` | `nv × nv` | 行主序，上下三角都填满 |
| `J` | `6 × nv` | 行主序。前 3 行线速度，后 3 行角速度 |
| `f_ext` | `6 × n_nodes` | 每个连杆一个空间力，连杆自身坐标系 |
| `T` | 16 | 4×4 齐次变换矩阵，**列主序** |

## 四元数顺序

`q_base[3..6]` 是 `[qw, qx, qy, qz]`，**标量在前**。

```c
q_base[0] = x;   q_base[1] = y;   q_base[2] = z;
q_base[3] = qw;  q_base[4] = qx;  q_base[5] = qy;  q_base[6] = qz;
```

ROS 和 Eigen 用的是 `[x, y, z, w]`，标量在后。顺序装反后模长仍是 1，
所有检查都能通过，结果是姿态错误的机器人。

## 空间向量

六维的速度、加速度、力统称空间向量，本库中**线量在前**：

```
速度  v = [vx, vy, vz,  wx, wy, wz]
力    f = [fx, fy, fz,  tx, ty, tz]
```

雅可比的 6 行同此顺序。Featherstone 的书中角量在前，抄书上的公式时需要调整。

## 参考系

rd_frame_t 有两个取值：

| 取值 | 坐标轴 | 取矩点 |
|---|---|---|
| `RD_FRAME_LOCAL` | 该 frame 自身 | 该 frame 原点 |
| `RD_FRAME_WORLD` | 世界系 | **世界原点** |

`RD_FRAME_WORLD` 对应 Pinocchio 的 `ReferenceFrame.WORLD`，**不是**
`LOCAL_WORLD_ALIGNED`。两者坐标轴相同，取矩点不同，只在 frame 原点与世界原点重合时
才相等。

多数应用需要的是 LOCAL_WORLD_ALIGNED（"这个点在世界系里往哪动、多快"）。
从 `RD_FRAME_WORLD` 的结果转换过去：

```c
/* J 是 rd_jacobian(..., RD_FRAME_WORLD, J) 的输出，行主序 6 x nv。
   p 是目标点在世界系下的坐标。 */
for (int c = 0; c < nv; ++c) {
    rd_real_t wx = J[3*nv + c], wy = J[4*nv + c], wz = J[5*nv + c];
    J[0*nv + c] -= p[1]*wz - p[2]*wy;
    J[1*nv + c] -= p[2]*wx - p[0]*wz;
    J[2*nv + c] -= p[0]*wy - p[1]*wx;
}
```

加速度还需要额外一项。点的经典加速度 = 平移后的空间加速度 + ω × ṗ。
rd_spatial_acceleration() 返回空间加速度，叉乘项由调用方补。

## 基座速度的参考系

`qd[0..5]`、`qdd[0..5]`、`tau[0..5]` 都表达在**根连杆自身坐标系**中，
对应 Pinocchio 的 free-flyer 约定。

状态估计器输出的通常是世界系速度旋量，放进 `qd` 之前需要旋转到根连杆坐标系。

`tau[0..5]` 的物理含义是作用在基座上的合力旋量。浮动基座没有驱动器，
所以这六行的非零值表示当前力矩没有满足的平衡条件，可用于判断力分配是否收敛。

## 重力

所有接受 `gravity` 参数的函数，传 `NULL` 表示世界系下的 `{0, 0, -9.81}`。

`rd_model_t::gravity` 字段**不会**被自动使用。要用模型里的值需要显式传入：

```c
rd_rnea(&chain, &state, qdd, (const rd_real_t*)&my_robot.gravity, tau);
```

rd_vec3_t 是三个连续的 rd_real_t，可以直接取址转换。

## 索引

| 类型 | 说明 |
|---|---|
| frame 索引 | `rd_idx_t`，`0` 到 `n_nodes - 1`。用 rd_chain_find_frame() 按名字获取 |
| 速度索引 | `rd_int_t`，`0` 到 `nv - 1`。rd_chain_set_armature() 用它 |
| 无父连杆 | `rd_link_t::parent_idx` 为 `-1` |
| 约束接地 | `rd_constraint_t::frame_b` 为 `RD_ANCHOR_WORLD`，值为 `-1` |

@warning `RD_ANCHOR_WORLD` 和 `RD_FRAME_WORLD` 是两个不同的东西。前者是 `rd_idx_t`，
值 `-1`，表示约束的另一端是世界；后者是 `rd_frame_t`，值 `0`，表示结果的参考系。
在 `frame_b` 位置写 `RD_FRAME_WORLD` 能编译通过，实际含义变成"约束到 0 号连杆"，
也就是基座。

固定连杆占用 frame 索引但不占速度索引。脚通常是固定连杆，可以对它求雅可比、加约束，
但它对 `nv` 无贡献。

## 关节顺序

按 URDF 声明顺序做深度优先遍历，与 Pinocchio 一致。`tools/urdf2c.py` 转换时会打印
完整映射表。运行时用 rd_chain_find_frame() 按名字取索引。

## 单位

全部为 SI 单位。

| 量 | 单位 |
|---|---|
| 长度 | m |
| 角度 | rad |
| 质量 | kg |
| 力 | N |
| 力矩 | N·m |
| 时间 | s |
| 转动惯量 | kg·m² |
| armature | 与 `M` 对角元同单位，转动关节为 kg·m² |
