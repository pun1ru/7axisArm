# 重力补偿符号约定与 Y(q) 说明

> 渲染约定：行内公式使用 `$...$`，独立公式使用 `$$...$$`；Python/C 代码仍放在代码块中。本文中的 $q$ 默认是数学编号，$q_1$ 对应固件 `J0`。

本文把重力补偿中容易混淆的符号、编号、矩阵维度和计算顺序统一说明。完整推导见根目录的 `重力补偿推导.md`。

## 0. 一句话结论

重力矩写成矩阵形式：

$$
\boldsymbol{\tau}_g(\boldsymbol{q})
=
\boldsymbol{Y}(\boldsymbol{q})\,\boldsymbol{\pi}
$$

- $\boldsymbol{q}$ 是关节角，会变化；
- $\boldsymbol{Y}(\boldsymbol{q})$ 是由 $\boldsymbol{q}$ 和 DH 参数算出来的已知矩阵；
- $\boldsymbol{\pi}$ 是待辨识参数向量，辨识完成后固定；
- 28 列版本是 $\boldsymbol{Y}(\boldsymbol{q})$ 乘 28 个参数；
- 12 列版本是 $\boldsymbol{B}(\boldsymbol{q})$ 乘 12 个基参数。

$\boldsymbol{\pi}$ 不是 $\boldsymbol{Y}(\boldsymbol{q})$。$\boldsymbol{\pi}$ 是列向量，$\boldsymbol{Y}(\boldsymbol{q})$ 是矩阵。

## 1. 编号约定

固件接口和上位机使用 `J0 ~ J6` 编号，数学推导使用从 1 开始的编号：

| 数学编号 | 固件编号 | 参数块 | 说明 |
|---:|---|---|---|
| 1 | J0 | $h_{1x},h_{1y},h_{1z},m_1$ | 基座竖直轴 |
| 2 | J1 | $h_{2x},h_{2y},h_{2z},m_2$ | |
| 3 | J2 | $h_{3x},h_{3y},h_{3z},m_3$ | |
| 4 | J3 | $h_{4x},h_{4y},h_{4z},m_4$ | |
| 5 | J4 | $h_{5x},h_{5y},h_{5z},m_5$ | |
| 6 | J5 | $h_{6x},h_{6y},h_{6z},m_6$ | |
| 7 | J6 | $h_{7x},h_{7y},h_{7z},m_7$ | |

对应关系为：

$$
q_1=\text{J0},\quad
q_2=\text{J1},\quad
\ldots,\quad
q_7=\text{J6}
$$

固件和上位机显示的角度单位是 deg；回归矩阵公式默认使用 rad：

$$
q_i^{\mathrm{rad}} = q_i^{\mathrm{deg}}\frac{\pi}{180}
$$

## 2. 单位和物理量

对第 $i$ 个连杆：

| 符号 | 含义 | 单位 |
|---|---|---|
| $m_i$ | 连杆质量 | kg |
| $\boldsymbol{c}_i=[c_{ix},c_{iy},c_{iz}]^T$ | 质心在连杆自身坐标系中的位置 | m |
| $\boldsymbol{h}_i=m_i\boldsymbol{c}_i$ | 质量加权质心，也叫一阶矩 | kg·m |
| $h_{ix},h_{iy},h_{iz}$ | $\boldsymbol{h}_i$ 的三个分量 | kg·m |
| $\tau_j$ | 关节 $j$ 的重力矩 | N·m |
| $g$ | 重力加速度，取 $9.81$ | m/s² |

每个连杆的参数向量是：

$$
\boldsymbol{\pi}_i
=
\begin{bmatrix}
h_{ix}\\
h_{iy}\\
h_{iz}\\
m_i
\end{bmatrix}
$$

整个机械臂的参数向量是 7 个 $\boldsymbol{\pi}_i$ 依次拼接，共 28 个元素：

$$
\boldsymbol{\pi}
=
\begin{bmatrix}
h_{1x},h_{1y},h_{1z},m_1,\;
h_{2x},h_{2y},h_{2z},m_2,\;
\ldots,\;
h_{7x},h_{7y},h_{7z},m_7
\end{bmatrix}^{T}
$$

这个 $\boldsymbol{\pi}$ 是待辨识参数，不是角度，也不是回归矩阵。

## 3. DH 参数和变换矩阵

采用改进 DH（Craig）：

$$
A_i
=
R_x(\alpha_{i-1})\,
D_x(a_{i-1})\,
R_z(q_i)\,
D_z(d_i)
$$

当前固件使用的理想 DH：

| 数学 $i$ | 固件 | $\alpha_{i-1}$ | $a_{i-1}$ | $d_i$ | $\theta_i$ |
|---:|---|---:|---:|---:|---|
| 1 | J0 | 0° | 0 | 0.0626 | $q_1$ |
| 2 | J1 | 90° | 0 | 0 | $q_2$ |
| 3 | J2 | 90° | 0 | -0.148 | $q_3$ |
| 4 | J3 | 90° | 0 | 0 | $q_4$ |
| 5 | J4 | 90° | 0 | 0.100 | $q_5$ |
| 6 | J5 | 90° | 0 | 0 | $q_6$ |
| 7 | J6 | 90° | 0 | 0 | $q_7$ |

定义：

$$
T_0=I_4,\qquad
T_i=A_1A_2\cdots A_i
$$

- $A_i$ 是相邻两个连杆坐标系之间的 $4\times4$ 齐次变换；
- $T_i$ 是从基座坐标系到第 $i$ 个连杆坐标系的 $4\times4$ 变换；
- $A_i$ 中含有 $R_z(q_i)$，所以 $A_i$、$T_i$ 和 $\boldsymbol{Y}(\boldsymbol{q})$ 都会随关节角变化。

## 4. Y(q) 怎么来的

第 $i$ 个连杆在基座坐标系中的质心位置记为 $\boldsymbol{p}_i$。重力势能为：

$$
U_i=m_i g\,p_{iz}
$$

定义选择向量：

$$
\boldsymbol{s}_z=
\begin{bmatrix}
0\\
0\\
1\\
0
\end{bmatrix}
$$

总势能可以写成：

$$
U(\boldsymbol{q})
=
g\sum_{i=1}^{7}
\boldsymbol{s}_z^{T}\,
T_i\,
\boldsymbol{\pi}_i
$$

其中 $\boldsymbol{s}_z^{T}T_i\boldsymbol{\pi}_i$ 正好取出质量乘质心的 $z$ 分量。

关节 $j$ 的重力矩由势能对关节角求导得到：

$$
\tau_j=\frac{\partial U}{\partial q_j}
$$

由于 $q_j$ 只出现在 $A_j$ 中，并且当 $j>i$ 时 $T_i$ 不含 $q_j$，所以：

$$
\boldsymbol{Y}_{j,\,4(i-1)+1:4i}
=
\begin{cases}
g\,\boldsymbol{s}_z^{T}
T_{j-1}
\dot A_j
A_{j+1}\cdots A_i,
& j\le i,\\[4pt]
\boldsymbol{0}_{1\times4},
& j>i,
\end{cases}
$$

其中：

$$
\dot A_j=\frac{\partial A_j}{\partial q_j}
$$

把每个关节 $j$ 作为一行，把每个连杆 $i$ 的 4 个参数作为连续的 4 列，就得到完整回归矩阵：

$$
\boldsymbol{Y}(\boldsymbol{q})\in\mathbb{R}^{7\times28}
$$

最终：

$$
\boldsymbol{\tau}_g(\boldsymbol{q})
=
\boldsymbol{Y}(\boldsymbol{q})\boldsymbol{\pi}
$$

### 4.1 行和列的含义

| 行 | 对应关节 | 说明 |
|---:|---|---|
| 1 | J0 | 重力矩恒为 0 |
| 2 | J1 | |
| 3 | J2 | |
| 4 | J3 | |
| 5 | J4 | |
| 6 | J5 | |
| 7 | J6 | |

| 列 | 参数 |
|---:|---|
| 1~4 | $h_{1x},h_{1y},h_{1z},m_1$ |
| 5~8 | $h_{2x},h_{2y},h_{2z},m_2$ |
| 9~12 | $h_{3x},h_{3y},h_{3z},m_3$ |
| 13~16 | $h_{4x},h_{4y},h_{4z},m_4$ |
| 17~20 | $h_{5x},h_{5y},h_{5z},m_5$ |
| 21~24 | $h_{6x},h_{6y},h_{6z},m_6$ |
| 25~28 | $h_{7x},h_{7y},h_{7z},m_7$ |

所以矩阵形式展开为：

$$
\begin{bmatrix}
\tau_{J0}\\
\tau_{J1}\\
\vdots\\
\tau_{J6}
\end{bmatrix}
=
\begin{bmatrix}
Y_{1,1} & \cdots & Y_{1,28}\\
Y_{2,1} & \cdots & Y_{2,28}\\
\vdots & \ddots & \vdots\\
Y_{7,1} & \cdots & Y_{7,28}
\end{bmatrix}
\begin{bmatrix}
h_{1x}\\
h_{1y}\\
\vdots\\
m_7
\end{bmatrix}
$$

$\boldsymbol{Y}(\boldsymbol{q})$ 的每一项都是 $\boldsymbol{q}$ 的三角函数组合，所以每个关节角变化时，整个 $\boldsymbol{Y}(\boldsymbol{q})$ 也会变化。不是 $\boldsymbol{\pi}$ 随 $\boldsymbol{q}$ 变化，而是 $\boldsymbol{Y}(\boldsymbol{q})$ 随 $\boldsymbol{q}$ 变化。

## 5. π 和 Y(q) 的区别

这两个东西必须分开：

- $\boldsymbol{\pi}$：28 个待辨识数字，是一列。
- $\boldsymbol{Y}(\boldsymbol{q})$：$7\times28$ 的矩阵，是 $\boldsymbol{q}$ 的函数。

具体关系是：

- $\boldsymbol{Y}(\boldsymbol{q})$ 的第 1 列对应 $h_{1x}$；
- $\boldsymbol{Y}(\boldsymbol{q})$ 的第 2 列对应 $h_{1y}$；
- $\cdots$
- $\boldsymbol{Y}(\boldsymbol{q})$ 的第 28 列对应 $m_7$。

因此：

$$
\boldsymbol{\tau}_g(\boldsymbol{q})
=
\boldsymbol{Y}(\boldsymbol{q})\boldsymbol{\pi}
$$

如果只用 12 列，则写成：

$$
\boldsymbol{B}(\boldsymbol{q})
=
\boldsymbol{Y}(\boldsymbol{q})[:,\mathcal{S}],
\qquad
\boldsymbol{\tau}_g(\boldsymbol{q})
=
\boldsymbol{B}(\boldsymbol{q})\boldsymbol{\beta}
$$

其中 $\mathcal{S}$ 是选中的 12 列集合，$\boldsymbol{\beta}$ 是对应的 12 个参数。

## 6. 秩 12 是怎么计算的

秩是矩阵的性质，不是 $\boldsymbol{\pi}$ 的性质。$\boldsymbol{\pi}$ 是列向量，没有“秩”这个概念。

对 $N$ 个不同位形，把 $\boldsymbol{Y}(\boldsymbol{q})$ 纵向堆起来：

$$
\boldsymbol{Y}_{\mathrm{stack}}
=
\begin{bmatrix}
\boldsymbol{Y}(\boldsymbol{q}^{(1)})\\
\boldsymbol{Y}(\boldsymbol{q}^{(2)})\\
\vdots\\
\boldsymbol{Y}(\boldsymbol{q}^{(N)})
\end{bmatrix}
\in\mathbb{R}^{7N\times28}
$$

然后做 SVD：

$$
\boldsymbol{Y}_{\mathrm{stack}}
=
\boldsymbol{U}\boldsymbol{\Sigma}\boldsymbol{V}^{T}
$$

数值秩按奇异值统计：

```python
import numpy as np

s = np.linalg.svd(Y_stack, compute_uv=False)
tol = max(Y_stack.shape) * np.finfo(float).eps * s[0]
rank = int(np.sum(s > tol))
```

按当前 DH 和 200 个限位内随机位形计算：

$$
\mathrm{rank}\left(\boldsymbol{Y}_{\mathrm{stack}}\right)=12
$$

前 12 个奇异值是几十到几百，第 13 个奇异值约为 $10^{-14}$，数值上就是零。

这表示：

$\boldsymbol{Y}_{\mathrm{stack}}$ 的 28 列中，只有 12 个独立的参数影响模式。

也就是说，28 个参数中只有 12 个独立组合会改变重力矩。存在一个维度为

$$
28-12=16
$$

的零空间。若 $\boldsymbol{n}$ 属于该零空间，则：

$$
\boldsymbol{Y}_{\mathrm{stack}}\boldsymbol{n}=0
$$

因此 $\boldsymbol{\pi}$ 和 $\boldsymbol{\pi}+\boldsymbol{n}$ 会产生完全相同的重力矩。

注意：如果只用一个位形，$\boldsymbol{Y}(\boldsymbol{q})$ 只有 7 行，秩最多是 6；必须使用足够多、角度差异足够大的位形，堆叠后才能达到结构上限 12。

## 7. 具体选哪 12 列

按当前 DH 数值，从完整的 28 列中选择下面 12 列。

### 7.1 参数名称

$$
\boldsymbol{\beta}
=
\begin{bmatrix}
h_{2x},h_{2y},\,
h_{3x},h_{3y},\,
h_{4x},h_{4y},\,
h_{5x},h_{5y},\,
h_{6x},h_{6y},\,
h_{7x},h_{7y}
\end{bmatrix}^{T}
$$

也就是连杆 2~7 的一阶矩在 $x,y$ 方向的分量。

### 7.2 列号

Python/C 的 0 基列号：

```python
SEL = [4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25]
```

如果从 1 开始数列号：

```text
5, 6, 9, 10, 13, 14, 17, 18, 21, 22, 25, 26
```

对应关系：

| $\boldsymbol{Y}$ 的列号（1 基） | 参数 | 说明 |
|---:|---|---|
| 5 | $h_{2x}$ | 连杆 2 |
| 6 | $h_{2y}$ | 连杆 2 |
| 9 | $h_{3x}$ | 连杆 3 |
| 10 | $h_{3y}$ | 连杆 3 |
| 13 | $h_{4x}$ | 连杆 4 |
| 14 | $h_{4y}$ | 连杆 4 |
| 17 | $h_{5x}$ | 连杆 5 |
| 18 | $h_{5y}$ | 连杆 5 |
| 21 | $h_{6x}$ | 连杆 6 |
| 22 | $h_{6y}$ | 连杆 6 |
| 25 | $h_{7x}$ | 连杆 7 |
| 26 | $h_{7y}$ | 连杆 7 |

代码：

```python
SEL = [4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25]
B = Y[:, SEL]       # 7 x 12
```

### 7.3 为什么选这 12 列

这组列不是随便挑的，而是对 $\boldsymbol{Y}_{\mathrm{stack}}$ 做列选择、QR 或 SVD 分析后得到的一组基：

- 连杆 1 的 4 列全为零，因为 J0 是竖直轴，重力不产生绕竖直轴的力矩；
- 质量项和 $h_{iz}$ 项在当前结构下和其他列线性相关；
- 连杆 2~7 的 $h_{ix},h_{iy}$ 这 12 列线性独立；
- 这 12 列可以张成完整 $\boldsymbol{Y}_{\mathrm{stack}}$ 的列空间。

数值验证为：

$$
\begin{aligned}
\boldsymbol{B}_{\mathrm{stack}}
&=
\boldsymbol{Y}_{\mathrm{stack}}[:,\mathcal{S}]
\in\mathbb{R}^{1400\times12},\\
\mathrm{rank}\left(\boldsymbol{B}_{\mathrm{stack}}\right)&=12,\\
\kappa\left(\boldsymbol{B}_{\mathrm{stack}}\right)&\approx3.40.
\end{aligned}
$$

用 $\boldsymbol{B}_{\mathrm{stack}}$ 重构完整 $\boldsymbol{Y}_{\mathrm{stack}}$：

```python
C = np.linalg.lstsq(B_stack, Y_stack, rcond=None)[0]
residual = B_stack @ C - Y_stack
```

最大重构误差约为：

$$
\max\left|\boldsymbol{B}_{\mathrm{stack}}\boldsymbol{C}
-\boldsymbol{Y}_{\mathrm{stack}}\right|
\approx3.55\times10^{-14}
$$

说明丢掉的其他 16 列没有提供额外信息。

这不是唯一的一组基；用 SVD 得到的 12 个基向量也可以。但这一组直接对应原始参数列，形式稀疏，适合单片机实现。

## 8. 拟合时到底拟合什么

采集数据时，每一个位形得到：

```text
q_k：7 个实际关节角
tau_k：7 个实际关节力矩
```

注意使用实际 `angle_deg`，不要使用 `target_deg`。

### 8.1 方案 A：拟合完整的 28 个参数

```python
Y_stack = np.vstack([
    gravity_regressor(q_k)       # 7 x 28
    for q_k in q_poses
])

tau_stack = np.concatenate([
    tau_k                        # 7
    for tau_k in tau_poses
])

pi_hat = np.linalg.lstsq(Y_stack, tau_stack, rcond=1e-8)[0]
```

对应的优化问题是：

$$
\hat{\boldsymbol{\pi}}
=
\arg\min_{\boldsymbol{\pi}}
\left\|
\boldsymbol{Y}_{\mathrm{stack}}\boldsymbol{\pi}
-\boldsymbol{\tau}_{\mathrm{stack}}
\right\|_2^2
$$

在线：

$$
\boldsymbol{\tau}_g(\boldsymbol{q})
=
\boldsymbol{Y}(\boldsymbol{q})\hat{\boldsymbol{\pi}}
$$

$\hat{\boldsymbol{\pi}}$ 是 28 个数。由于秩是 12，这个解不是唯一的；`lstsq` 返回的是最小范数解。

### 8.2 方案 B：直接拟合 12 个基参数（推荐）

```python
SEL = [4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25]

B_stack = Y_stack[:, SEL]        # (7N) x 12
beta_hat = np.linalg.lstsq(B_stack, tau_stack, rcond=None)[0]

# beta_hat 的顺序：
# [h2x, h2y, h3x, h3y, h4x, h4y,
#  h5x, h5y, h6x, h6y, h7x, h7y]
```

对应的优化问题是：

$$
\hat{\boldsymbol{\beta}}
=
\arg\min_{\boldsymbol{\beta}}
\left\|
\boldsymbol{B}_{\mathrm{stack}}\boldsymbol{\beta}
-\boldsymbol{\tau}_{\mathrm{stack}}
\right\|_2^2
$$

在线：

$$
\boldsymbol{\tau}_g(\boldsymbol{q})
=
\boldsymbol{B}(\boldsymbol{q})\hat{\boldsymbol{\beta}}
$$

其中：

$$
\boldsymbol{B}(\boldsymbol{q})
=
\boldsymbol{Y}(\boldsymbol{q})[:,\mathcal{S}]
\in\mathbb{R}^{7\times12}
$$

推荐直接使用方案 B，不需要先拟合 28 个参数。

### 8.3 如果已经有了 28 个参数怎么办

不能直接写：

$$
\boldsymbol{Y}(\boldsymbol{q})[:,\mathcal{S}]\,
\hat{\boldsymbol{\pi}}[\mathcal{S}]
$$

因为完整 $\boldsymbol{Y}(\boldsymbol{q})\hat{\boldsymbol{\pi}}$ 中，其他相关列也有贡献。正确做法是投影：

```python
pi_hat = np.linalg.lstsq(Y_stack, tau_stack, rcond=1e-8)[0]

B_stack = Y_stack[:, SEL]
beta_hat = np.linalg.lstsq(
    B_stack,
    Y_stack @ pi_hat,
    rcond=None
)[0]
```

对应公式：

$$
\hat{\boldsymbol{\beta}}
=
\arg\min_{\boldsymbol{\beta}}
\left\|
\boldsymbol{B}_{\mathrm{stack}}\boldsymbol{\beta}
-\boldsymbol{Y}_{\mathrm{stack}}\hat{\boldsymbol{\pi}}
\right\|_2^2
$$

这样：

$$
\boldsymbol{B}_{\mathrm{stack}}\hat{\boldsymbol{\beta}}
\approx
\boldsymbol{Y}_{\mathrm{stack}}\hat{\boldsymbol{\pi}}
$$

更简单的方式仍然是直接用实测力矩拟合 $\boldsymbol{\beta}$：

```python
beta_hat = np.linalg.lstsq(B_stack, tau_stack, rcond=None)[0]
```

## 9. SVD 的 12 个基和这 12 列不要混用

另一种 12 参数形式是：

```python
U, s, Vt = np.linalg.svd(Y_stack, full_matrices=False)
Vr = Vt[:12].T
Yr = Y_stack @ Vr
beta = np.linalg.lstsq(Yr, tau_stack)[0]
```

对应的数学形式是：

$$
\boldsymbol{Y}_{\mathrm{stack}}
=
\boldsymbol{U}\boldsymbol{\Sigma}\boldsymbol{V}^{T},
\qquad
\boldsymbol{V}_r
=
\boldsymbol{V}[:,\,1:12]
$$

$$
\boldsymbol{Y}_{r,\mathrm{stack}}
=
\boldsymbol{Y}_{\mathrm{stack}}\boldsymbol{V}_r
$$

这里的 $\boldsymbol{\beta}$ 是 SVD 基参数，和本文选出的 12 个物理列不是同一套坐标。

两套都可以，但不要混用：

- 用本文的 $\mathcal{S}$，就存 $\boldsymbol{B}(\boldsymbol{q})\hat{\boldsymbol{\beta}}$；
- 用 SVD 的 $\boldsymbol{V}_r$，就存 $\boldsymbol{Y}_r(\boldsymbol{q})\hat{\boldsymbol{\beta}}$ 对应的 12 个系数；
- 用完整 28 参数，就存 $\boldsymbol{Y}(\boldsymbol{q})\hat{\boldsymbol{\pi}}$。

## 10. 单片机在线怎么省算力

最省力的方式是只计算 12 列基回归矩阵 $\boldsymbol{B}(\boldsymbol{q})$，并且直接累加力矩，不生成完整 $\boldsymbol{Y}(\boldsymbol{q})$。

### 10.1 关键事实

- 固件 `J0` 的重力矩恒为 0，$q_0$ 不参与重力计算；
- 选中的 12 个参数只涉及 $h_x,h_y$；
- 这些参数只依赖旋转矩阵，不依赖 DH 的平移量 $d$；
- 因此在线不需要 $4\times4$ 齐次矩阵，只需要 $3\times3$ 旋转矩阵。

### 10.2 递推公式

对固件 `J1 ~ J6` 对应的 $q_1\sim q_6$（这里的下标按固件从 1 开始）：

$$
A_i=
\begin{bmatrix}
c_i & -s_i & 0\\
0 & 0 & -1\\
s_i & c_i & 0
\end{bmatrix},
\qquad
c_i=\cos q_i,\quad s_i=\sin q_i
$$

$$
R_0=I_3,
\qquad
R_i=R_{i-1}A_i
$$

对每个关节 $j=1,\ldots,6$：

$$
\boldsymbol{v}
=
\begin{bmatrix}
(R_j)_{32}\\
-(R_j)_{31}\\
0
\end{bmatrix}
$$

然后对 $i=j,\ldots,6$ 累加：

$$
\tau_j \leftarrow \tau_j
+g\left(
v_1\beta_{2(i-1)+1}
+v_2\beta_{2(i-1)+2}
\right)
$$

其中 $\boldsymbol{\beta}$ 的顺序是

$$
[h_{2x},h_{2y},h_{3x},h_{3y},\ldots,h_{7x},h_{7y}]^{T}.
$$

若 $i<6$，更新：

$$
\boldsymbol{v}\leftarrow\boldsymbol{v}A_{i+1}
$$

最后：

$$
\tau_{J0}=0
$$

整个周期只需要：

- 6 组 `sinf/cosf`；
- 6 次 $3\times3$ 旋转递推；
- 15 次 3 维行向量乘旋转矩阵；
- 若干次乘加。

## 11. 常见问题

### Q1：Y(q) 是 7×28 还是 7×12？

两套都可以：

完整形式：

$$
\boldsymbol{Y}(\boldsymbol{q})\in\mathbb{R}^{7\times28},
\qquad
\boldsymbol{\tau}=\boldsymbol{Y}\boldsymbol{\pi}
$$

基参数形式：

$$
\boldsymbol{B}(\boldsymbol{q})\in\mathbb{R}^{7\times12},
\qquad
\boldsymbol{\tau}=\boldsymbol{B}\boldsymbol{\beta}
$$

$\boldsymbol{B}(\boldsymbol{q})$ 就是 $\boldsymbol{Y}(\boldsymbol{q})$ 的 12 个列。

### Q2：π 为什么和 q 无关？

$\boldsymbol{\pi}$ 是连杆的物理/等效参数；在刚性机械臂上，它不随关节角变化。随 $q$ 变化的是回归矩阵 $\boldsymbol{Y}(\boldsymbol{q})$，因为其中有 $q$ 的三角函数。

### Q3：秩 12 是对 Y(q) 还是对 π？

是对矩阵 $\boldsymbol{Y}(\boldsymbol{q})$ 或 $\boldsymbol{B}(\boldsymbol{q})$ 的堆叠矩阵求秩。$\boldsymbol{\pi}$ 是列向量，没有秩。

### Q4：能不能把所有 28 个参数都拟合出来？

如果指“让预测力矩贴合实测力矩”：可以。  
如果指“唯一恢复每个连杆的真实质量和质心”：不行。秩只有 12，存在 16 维零空间，无穷多组参数产生同样力矩。

### Q5：为什么不能直接用 Y[:,S]π[S]？

因为完整 $\boldsymbol{Y}\boldsymbol{\pi}$ 中，其他列也有贡献。若已经有 28 维 $\boldsymbol{\pi}$，要先用 $\boldsymbol{B}_{\mathrm{stack}}$ 对 $\boldsymbol{Y}_{\mathrm{stack}}\boldsymbol{\pi}$ 做投影。

### Q6：这 12 列是永远固定的吗？

对当前 DH 和机械结构，按数值验证是固定的。修改 DH、连杆结构或增加不同末端负载后，应重新做一次 $\boldsymbol{Y}_{\mathrm{stack}}$ 的秩检查和列选择。末端负载变化时，即使 12 列不变，$\boldsymbol{\beta}$ 也会变化，需要重新辨识。

## 12. 最小验算代码

```python
import numpy as np

# 当前固件理想 DH: alpha_deg, a_m, d_m
DH = [
    (0.0,  0.0,  0.0626),
    (90.0, 0.0,  0.0),
    (90.0, 0.0, -0.148),
    (90.0, 0.0,  0.0),
    (90.0, 0.0,  0.100),
    (90.0, 0.0,  0.0),
    (90.0, 0.0,  0.0),
]

SEL = [4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25]


def gravity_regressor(q_deg, g=9.81):
    q = np.radians(np.asarray(q_deg, float))
    A, dA = [], []

    for (alpha_deg, a, d), qi in zip(DH, q):
        alpha = np.radians(alpha_deg)
        c, s = np.cos(qi), np.sin(qi)
        ca, sa = np.cos(alpha), np.sin(alpha)

        A.append(np.array([
            [c, -s, 0.0, a],
            [s * ca, c * ca, -sa, -sa * d],
            [s * sa, c * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0],
        ]))

        dA.append(np.array([
            [-s, -c, 0.0, 0.0],
            [c * ca, -s * ca, 0.0, 0.0],
            [c * sa, -s * sa, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ]))

    T = [np.eye(4)]
    for Ai in A:
        T.append(T[-1] @ Ai)

    sz = np.array([0.0, 0.0, 1.0, 0.0])
    Y = np.zeros((7, 28))

    for j in range(7):
        P = T[j] @ dA[j]
        for i in range(j, 7):
            M = P.copy()
            for k in range(j + 1, i + 1):
                M = M @ A[k]
            Y[j, 4 * i:4 * i + 4] = g * (sz @ M)

    return Y


if __name__ == "__main__":
    rng = np.random.default_rng(2026)
    limits = np.array([
        [-90, 90], [-95, 100], [-180, 180], [-120, 0],
        [-180, 180], [-60, 90], [-180, 180],
    ], float)

    qs = rng.uniform(limits[:, 0] + 8, limits[:, 1] - 8, size=(200, 7))
    Y_stack = np.vstack([gravity_regressor(q) for q in qs])
    sY = np.linalg.svd(Y_stack, compute_uv=False)
    rankY = np.sum(sY > max(Y_stack.shape) * np.finfo(float).eps * sY[0])

    B_stack = Y_stack[:, SEL]
    sB = np.linalg.svd(B_stack, compute_uv=False)
    rankB = np.sum(sB > max(B_stack.shape) * np.finfo(float).eps * sB[0])

    C = np.linalg.lstsq(B_stack, Y_stack, rcond=None)[0]
    residual = np.max(np.abs(B_stack @ C - Y_stack))

    print("Y_stack:", Y_stack.shape, "rank:", rankY)
    print("B_stack:", B_stack.shape, "rank:", rankB)
    print("B singular values:", np.round(sB, 6))
    print("B condition:", sB[0] / sB[-1])
    print("max reconstruction residual:", residual)
```

## 13. 由 `robot.xml` 得到的名义参数（对照用）

以下数值直接从 `Simulation/mujoco/robot.xml` 的 `inertial` 读取，只使用质量和质心位置。MuJoCo 的
`inertial quat` 只描述惯性主轴方向，不参与重力项，因此计算重力参数时不需要。

```text
mass_i      <- <inertial mass="...">
c_i^body    <- <inertial pos="...">
```

固定基座 `base` 不进入参数向量。第 $i$ 个连杆的质心先由 XML 的 body 链正运动学转到世界系，再转到
q = 0 时的第 $i$ 个 DH 坐标系：

$$
\boldsymbol{c}_i^{\mathrm{DH}}
=
\left(\boldsymbol{R}_i^{\mathrm{DH}}\right)^{T}
\left(
\boldsymbol{p}_{i,\mathrm{COM}}^{\mathrm{world}}
-
\boldsymbol{p}_i^{\mathrm{DH}}
\right)
$$

$$
\boldsymbol{h}_i=m_i\boldsymbol{c}_i^{\mathrm{DH}},
\qquad
\boldsymbol{\pi}=
\left[h_{1x},h_{1y},h_{1z},m_1,\ldots,h_{7x},h_{7y},h_{7z},m_7\right]^T
$$

### 13.1 精确 DH 下的名义 $\pi$

精确 DH 使用 `dh_params.md` 第 1 节的 d：
`0.0626, -0.00400004092454, -0.147999608, -0.00249973171439, 0.0999988885966, -0.000999402709942, 0`。

| 连杆 | $m_i$ [kg] | $c_i^{\mathrm{DH}}$ [m] |
|---|---:|---|
| 1 | 0.575927632 | (-0.0007063516978, -0.001009648166, -0.06828822015) |
| 2 | 0.452797436 | (0.0000453598597332, 0.0049215465308, 0.0157847614606) |
| 3 | 0.580566942 | (0.000444989480972, 0.0057004911723, 0.0607743396302) |
| 4 | 0.336674971 | (-0.0000360417509836, -0.0016147434978, 0.00255977030518) |
| 5 | 0.144687183 | (0.0000762130986343, 0.00336965916926, -0.0410908758906) |
| 6 | 0.144933556 | (0.0000319662969854, 0.00246760121436, 0.00615557494411) |
| 7 | 0.0736087532 | (0.00000216514629339, -0.0000840638731065, -0.0472675208593) |

按连杆顺序 `[hx, hy, hz, m]` 拼接得到：

```python
pi_robot_xml_exact_dh = [
    -0.000406807460673, -0.000581484277398, -0.0393290729245, 0.575927632,
     0.0000205388281845, 0.0022284636503, 0.00714729951725, 0.452797436,
     0.00025834618219, 0.0033095167278, 0.0352835725112, 0.580566942,
    -0.0000121343554672, -0.000543643720294, 0.000861810593263, 0.336674971,
     0.0000110270585491, 0.000487546492871, -0.00594532307962, 0.144687183,
     0.00000463298909425, 0.000357638218787, 0.000892149365875, 0.144933556,
     0.000000159373719152, -0.00000618783688853, -0.00347930327731, 0.0736087532,
]
```

### 13.2 固件理想 DH 下的名义 $\pi$

固件 `module_init.c` 和上位机默认 DH 用的是理想版：
`d = [0.0626, 0, -0.148, 0, 0.100, 0, 0]`。
同一组 XML 质量、质心换到理想 DH 帧后为：

```python
pi_robot_xml_ideal_dh = [
    -0.000406807460673, -0.000581484277398, -0.0393290729245, 0.575927632,
     0.0000205388281845, 0.0022284636503, 0.00533609124272, 0.452797436,
     0.00025834618219, 0.000987225200368, 0.0352838000934, 0.580566942,
    -0.0000121343554672, -0.000543511743706, 0.00136692715308, 0.336674971,
     0.0000110270585491, 0.000704622006117, -0.00594554060282, 0.144687183,
     0.00000463298909425, 0.000357420325186, 0.000529857228321, 0.144933556,
     0.000000159373719152, -0.000190188514686, -0.00347919261366, 0.0736087532,
]
```

注意：理想 DH 的 4/2.5/1 mm 偏置被置零后，`Y_ideal @ pi_robot_xml_ideal_dh` 与 `robot.xml`
真实几何的重力矩并不完全相等，在 500 个随机位形上最大差约 `0.049 N·m`。因此它只用于检查
名义量级；真正给控制器做重力补偿时，应使用下面投影得到的 12 维 $\beta$。

### 13.3 真正适合和辨识结果对照的 12 维 $\beta$

$\pi$ 的 28 个分量不是唯一可辨识的。若辨识程序按默认方式调用
`np.linalg.lstsq(Y, tau)`，它返回的是最小范数解，**不应把返回的 28 个数逐项和上面的物理
$\pi$ 比较**。应先比较力矩预测，或比较下面的基参数：

$$
\mathrm{SEL}=[4,5,8,9,12,13,16,17,20,21,24,25]
$$

对应列：

```text
h2x, h2y, h3x, h3y, h4x, h4y, h5x, h5y, h6x, h6y, h7x, h7y
```

用 `beta_ref = lstsq(B_stack, Y_stack @ pi_ref)` 投影得到：

```python
beta_robot_xml = [
     0.0000205388281845, 0.156454157164,
     0.00025834618219,   0.00419727951824,
    -0.0000121343554673, -0.0309208661662,
     0.0000110270585491, -0.000186191096952,
     0.00000463298909436, 0.0038369414961,
     0.000000159373719152, -0.0000061878368885,
]
```

在随机位形上，$\boldsymbol{B}\boldsymbol{\beta}$ 与 `Y @ pi_robot_xml_exact_dh` 的最大重构误差
约为 `1.6e-15 N·m`。12 列堆叠矩阵的条件数约 `3.4`。精确 DH 和固件理想 DH 下，这组 12 维
$\beta$ 到机器精度相同。

### 13.4 读数注意

- 上表使用 $g=9.81\ \mathrm{m/s^2}$，与当前 XML 的 MuJoCo 默认重力一致。
- `pi` 中的前三个分量单位是 kg·m，最后一个分量单位是 kg。
- `robot.xml` 的固定 body `base` 与运动 body `link1` 的 `inertial` 数值相同；计算时只计入
  `link1`，固定 `base` 不计入。由于 J0 轴竖直，前四个参数对重力矩的贡献为 0，所以辨识秩中
  不包含它们。
- 这是 CAD/XML 名义值，不是实机辨识值。减速器摩擦、零位误差、线缆和末端负载都会使实机结果
  偏离。
- 若 CSV 中记录的力矩与 $\partial U/\partial q$ 反号，上位机拟合时选 `sign=-1`，对应的
  $\beta$ 也整体取负。
