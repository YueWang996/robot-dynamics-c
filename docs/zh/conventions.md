# 约定 {#conventions}

这页上的每一条弄错了都不会报错，只会给出一个看起来合理的错误答案。而且运行时查不出来 ——
一个顺序反了的四元数和一个正确的四元数，都是单位四元数。

这些约定和 [Pinocchio](https://github.com/stack-of-tasks/pinocchio)、
[bard](https://github.com/YueWang996/bard-pytorch-dynamics) 一致，所以为它们写的 `q`
向量可以直接传给这个库，反过来也一样。

## 一句话版本

| | |
|---|---|
| 基座位姿 | `q_base = [x y z qw qx qy qz]`，**标量在前** |
| 基座速度与加速度 | 根连杆的**体坐标系** |
| 空间向量 | `[线量, 角量]` |
| `qd`、`qdd`、`tau` | 长度 `nv`，紧凑排布，基座占前六位 |
| `M` | `nv × nv`，两个三角都填满 |
| 关节顺序 | 按 URDF 声明顺序深度优先 |
| 重力 | 传 `NULL` 表示世界系下的 `{0, 0, -9.81}` |

## 位形是拆开的，速度不是

浮动基座下 `nq` 和 `nv` 不相等，因为表示三个转动自由度需要一个四元数。所以位形是两个
参数，速度是一个：

```c
rd_real_t q_base[7];         /* x y z qw qx qy qz  —— 标量在前            */
rd_real_t q_joints[nj];      /* 每个驱动关节一个                          */
rd_real_t qd[nv];            /* 浮动基座 nv = 6 + nj，固定基座 nv = nj    */
```

固定基座的机器人给 `q_base` 传 `NULL`，基座就放在单位位姿上。`q_joints` 传 `NULL`
得到零位形，`qd` 传 `NULL` 表示静止。

标量在前的四元数是最该复核的一条。ROS 和 Eigen 都写成 `[x y z w]`，而顺序装错的四元数
仍然是单位四元数，所以任何地方都不会报错。你得到的是一个姿态莫名其妙的机器人。

## 基座速度在根连杆体坐标系里

`qd[0..5]` 是基座的空间速度，表达在根连杆自己的坐标系里，这是 Pinocchio 的 free-flyer
约定。它不是基座原点在世界系下的速度。如果你的状态估计器给你的是世界系的速度旋量，
先把它转到根坐标系再放进 `qd`。

`qdd[0..5]` 和 `tau` 的前六位同理，后者是作用在基座上的合力旋量。浮动基座没有驱动器，
所以这六行里剩下什么，就是还没被满足的那个平衡条件。@ref contacts "接触与闭链" 里有一个实际的例子。

## 空间向量是 [线量, 角量]

六元的空间量，以及雅可比的各行，线量在前：

```
v = [vx vy vz  wx wy wz]
f = [fx fy fz  tx ty tz]
```

Featherstone 的书里角量在前。这个库跟 Pinocchio。

## "世界系"到底指什么

rd_frame_t 只有两个取值，会被读错的是 RD_FRAME_WORLD。

- `RD_FRAME_LOCAL` —— 在该 frame 自己的体坐标轴下，取在它自己的原点。
- `RD_FRAME_WORLD` —— Pinocchio 的 `ReferenceFrame.WORLD`：空间向量取在**世界原点**，
  用世界坐标轴。

`RD_FRAME_WORLD` 和 `LOCAL_WORLD_ALIGNED` 是两回事。只有当该 frame 的原点正好落在世界
原点上时两者才相同，差的那一项随 frame 离原点的距离增长 —— 所以这个错误在小机器人上
表现为小误差，在腿上就是大误差。

要拿到点 `p` 处的世界系对齐量，把 WORLD 的那个平移过去：

```c
rd_jacobian(&chain, &state, frame, RD_FRAME_WORLD, J);
/* 行主序，6 x nv。把线量三行平移到点 p： */
for (int c = 0; c < nv; ++c) {
    rd_real_t wx = J[3*nv + c], wy = J[4*nv + c], wz = J[5*nv + c];
    J[0*nv + c] -= p[1]*wz - p[2]*wy;
    J[1*nv + c] -= p[2]*wx - p[0]*wz;
    J[2*nv + c] -= p[0]*wy - p[1]*wx;
}
```

加速度还多一项。一个点的经典加速度等于平移后的空间加速度加上 ω × ṗ；
rd_spatial_acceleration() 返回的是空间加速度，那个叉乘项由调用方补。

## RD_ANCHOR_WORLD 不是 RD_FRAME_WORLD

这两个名字长得像，含义毫无关系，而编译器不会把它们分开。

| | |
|---|---|
| RD_FRAME_WORLD | 是 rd_frame_t。说的是雅可比或速度*表达在哪个系*。取值 0。 |
| RD_ANCHOR_WORLD | 是 rd_idx_t。说的是约束的第二个 frame *就是世界*。取值 -1。 |

给约束的 `frame_b` 写 RD_FRAME_WORLD 能编译通过，然后安静地把脚约束到 0 号连杆 ——
也就是基座 —— 因为那个枚举的零值当索引用就是这个意思。于是脚跟着身体一起动，
求解返回的数字看起来还挺像样。

## 关节顺序

按 URDF 声明关节的顺序深度优先，和 Pinocchio 一致。`tools/urdf2c.py` 转换模型时会
把映射打出来，运行时用 rd_chain_find_frame() 按名字拿索引。

固定连杆占一个 frame 索引，但不占速度索引。脚通常就是固定连杆：它有索引，可以对它求
雅可比，但它对 `nv` 没有贡献。

## 质量矩阵是填满的

rd_crba() 把整个 `nv × nv` 矩阵两个三角都写满，行主序。可以直接传给
rd_cholesky_factor()，后者只读下三角。

## 单位

全程 SI。米、弧度、千克、牛顿、牛·米、秒。惯量是 kg·m²，armature 的单位和 `M` 的对角
元一致，转动关节下就是 kg·m²。
