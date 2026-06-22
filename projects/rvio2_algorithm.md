---
title: "R-VIO2：平方根机器人中心视觉-惯性里程计算法详解"
date: 2026-06-22
tags: [R-VIO2 SLAM VIO 视觉惯性里程计 机器人中心 平方根信息滤波]
gitalk: false
---

# R-VIO2：平方根机器人中心视觉-惯性里程计算法详解

> **论文引用**：
> - Zheng Huai and Guoquan Huang, *Square-Root Robocentric Visual-Inertial Odometry with Online Spatiotemporal Calibration*, IEEE RA-L, 2022.
> - Zheng Huai and Guoquan Huang, *A Consistent Parallel Estimation Framework for Visual-Inertial SLAM*, IEEE T-RO, 2024.
>
> **代码仓库**：[github.com/rpng/R-VIO2](https://github.com/rpng/R-VIO2)

---

## 1. 算法概述

R-VIO2 是一种基于**平方根信息矩阵**的**机器人中心（Robocentric）**视觉-惯性导航算法，使用单目相机和单个 IMU 实现一致的 3D 运动跟踪。其核心创新包括：

1. **Robocentric 状态表示**：所有状态变量在机器人当前局部坐标系 $\{I_k\}$ 中表示，而非世界坐标系；
2. **平方根信息滤波**：维护上三角平方根信息矩阵 `LocalFactor`（记为 $\mathbf{R}$），满足 $\boldsymbol{\Lambda} = \mathbf{R}^\top \mathbf{R}$，条件数减半；
3. **QR 消元 + 回代更新**：通过 Givens 旋转将新观测吸收进 $\mathbf{R}$，利用回代法求解状态修正量；
4. **在线时空标定**：同时估计相机-IMU 外参 $(\mathbf{R}_{CI}, \mathbf{t}_{CI})$ 和时间偏移 $t_d$。

算法支持两种模式：**VIO**（仅估计滑动窗口内的相对位姿）和 **SLAM**（额外维护稀疏地图点用于定位与建图）。

---

## 2. 系统架构

### 2.1 模块组成

```
┌─────────────────────────────────────────────────────┐
│                    System (主循环)                    │
│                                                      │
│  InputBuffer → Propagator → Tracker → Updater       │
│   (数据缓冲)   (IMU传播)  (视觉跟踪)  (视觉更新)       │
└─────────────────────────────────────────────────────┘
```

| 模块 | 文件 | 职责 |
|------|------|------|
| `System` | `System.cc/h` | 主循环：Propagate → Track → Update → Composition |
| `Propagator` | `Propagator.cc/h` | IMU 数值积分 + 协方差传播 + 平方根信息因子构建 |
| `Tracker` | `Tracker.cc/h` | KLT 光流跟踪 + RANSAC + 特征管理 |
| `Updater` | `Updater.cc/h` | 视觉观测更新：三角化 + 多类特征处理 + QR 消元 + 状态更新 |
| `FeatureDetector` | `FeatureDetector.cc/h` | Shi-Tomasi 角点检测 + 网格均匀分布 |
| `Ransac` | `Ransac.cc/h` | 5-point RANSAC 本质矩阵估计 |
| `InputBuffer` | `InputBuffer.cc/h` | IMU/图像数据时间对齐缓冲 |
| `numerics.h` | `util/numerics.h` | 四元数/旋转矩阵运算 + Chi-square 查找表 |

### 2.2 主循环流程

```cpp
// System.cc:294 — System::run()
void System::run() {
    // 1. 获取时间对齐的 IMU + 图像数据
    mpInputBuffer->GetMeasurements(mnCamTimeOffset, pMeasurements);

    // 2. 初始化（仅一次）：静止检测 + 重力估计 + bias 估计
    if (!mbIsInitialized) initialize(...);

    // 3. IMU 传播：积分 + 协方差预测 + 平方根信息因子
    mpPropagator->propagate(nImageId, vImuData, Localx, LocalFactor);

    // 4. 预测相机位姿
    RcG = mRci * Rk * RG;
    tcG = mRci * Rk * (pG - tk) + tci;

    // 5. 视觉跟踪：光流 + RANSAC + 特征分类
    mpTracker->track(nImageId, image, RcG, tcG, nMapPtsNeeded, mmFeatures);

    // 6. 视觉更新：吸收三类观测 + 状态回代更新
    mpUpdater->update(nImageId, mmFeatures, ...,
                      mvActiveFeatureIDs, mqLocalw, mqLocalv,
                      Localx, LocalFactor);
}
```

---

## 3. 状态表示

### 3.1 Robocentric 状态向量

所有状态均在当前 IMU 坐标系 $\{I_k\}$ 中表示（`System.h:70`）：

$$
\mathbf{x} = \begin{bmatrix}
{}^G_{I_k}\bar{q} \\ \mathbf{p}_{G/I_k} \\ {}^G\mathbf{g} \\ {}^{I_k}_{C}\bar{q} \\ \mathbf{p}_{C/I_k} \\ t_d \\ {}^{I_k}\mathbf{v} \\ \mathbf{b}_g \\ \mathbf{b}_a \\ \boldsymbol{\xi}_1 \\ \vdots \\ \boldsymbol{\xi}_N
\end{bmatrix}
\quad
\begin{aligned}
&\text{全局姿态（四元数）} &4 \\
&\text{全局位置（局部坐标系中的世界原点）} &3 \\
&\text{局部重力方向} &3 \\
&\text{IMU→Camera 外参旋转} &4 \\
&\text{IMU→Camera 外参平移} &3 \\
&\text{时间偏移} &1 \\
&\text{局部速度} &3 \\
&\text{陀螺仪 bias} &3 \\
&\text{加速度计 bias} &3 \\
&\text{滑动窗口：N 个相对位姿 } (q_{C_{k-i}}, t_{C_{k-i}}) &7N
\end{aligned}
$$

```cpp
// 代码对应 — System.cc:211
Localx.setZero(27);  // 初始维度：4+3+3+4+3+1+3+3+3 = 27
// 每帧传播后增加 7 维（一个相对位姿）
```

**关键坐标关系**：

- 世界到局部：$\mathbf{p}_{G/I_k} = {}^{I_k}_{G}\mathbf{R}^\top (-\mathbf{p}_{I_k/G})$
- 相机到世界：${}^{C_k}_G\mathbf{R} = \mathbf{R}_{CI} \mathbf{R}_{I_k G}$，$\mathbf{t}_{C_k/G} = \mathbf{R}_{CI} \mathbf{t}_{I_k/G} + \mathbf{t}_{CI}$

### 3.2 平方根信息矩阵

`LocalFactor`（记为 $\mathbf{R}$）是一个上三角矩阵，满足：

$$
J(\tilde{\mathbf{x}}) = \|\mathbf{R} \delta\mathbf{x} - \mathbf{d}\|^2
$$

其中 $\tilde{\mathbf{x}} = \mathbf{x} \boxplus \delta\mathbf{x}$（$\boxplus$ 表示流形上的广义加法）。

```cpp
// System.h:73
Eigen::MatrixXf LocalFactor;  // 上三角平方根信息矩阵
// 结构：[R | d]，最后一列是信息向量
// 维度：L × (L+1)，其中 L 随状态增长
```

**与协方差/信息矩阵的关系**：

| 表示 | 符号 | 关系 | 条件数 | 存储 |
|------|:----:|------|:------:|------|
| 协方差矩阵 | $\mathbf{P}$ | — | $\kappa(\mathbf{P})$ | ❌ 不存 |
| 信息矩阵 | $\boldsymbol{\Lambda} = \mathbf{P}^{-1}$ | $\boldsymbol{\Lambda} = \mathbf{R}^\top\mathbf{R}$ | $\kappa(\boldsymbol{\Lambda})$ | ❌ 不存 |
| **平方根信息矩阵** | $\mathbf{R}$（上三角） | — | $\sqrt{\kappa(\boldsymbol{\Lambda})}$ | ✅ `LocalFactor` |

---

## 4. 初始化

### 4.1 静止检测

```cpp
// System.cc:127 — System::initialize()
// 累积 IMU 数据，计算累积角度和位移
for (const ImuData& data : vImuData) {
    ang += dt * (w_m);                          // 累积角速度
    vel += dt * (a_m - g * a_m/|a_m|);         // 去重力后的速度
    len += dt * vel + 0.5 * dt² * (a_m - g * a_m/|a_m|); // 累积位移
}
// 判据：角度 < mnAngleThrd 且位移 < mnLengthThrd → 静止
if (ang.norm() < threshold && len.norm() < threshold) return false;
```

### 4.2 初始状态估计

检测到运动后，利用静止期间的 IMU 数据估计初始状态：

$$
\mathbf{b}_g = \frac{1}{N}\sum \boldsymbol{\omega}_m, \quad
\mathbf{b}_a = \frac{1}{N}\sum \mathbf{a}_m - g \cdot \frac{\sum \mathbf{a}_m}{\|\sum \mathbf{a}_m\|}
$$

```cpp
// System.cc:194-209
wm /= nImuCount;  am /= nImuCount;
g = am;  g.normalize();        // 重力方向 = 平均加速度方向
bg = wm;                        // 初始陀螺仪 bias
ba = am - mnGravity * g;        // 初始加速度计 bias
```

### 4.3 初始先验信息矩阵

```cpp
// System.cc:238-265 — LocalFactor 对角线元素 = 1/σ
LocalFactor(0,0) = 1./1e-6;                            // qG:  σ=1e-6
LocalFactor(6,6) = 1./sqrt(Dt)/msigmaAccelNoise;       // g:   离散随机游走
LocalFactor(9,9) = mbUseGroundTruthCalib ? 1./2e-2 : 1./2e-1; // qCI 外参
LocalFactor(19,19) = 1./sqrt(Dt)/msigmaGyroBias;       // bg 随机游走
```

---

## 5. IMU 传播（Propagator）

### 5.1 运动方程

IMU 测量模型（`Propagator.cc:95-98`）：

$$
\boldsymbol{\omega}_m = \boldsymbol{\omega} + \mathbf{b}_g + \mathbf{n}_g, \quad
\mathbf{a}_m = \mathbf{a} + \mathbf{b}_a + \mathbf{n}_a
$$

状态微分方程（robocentric 形式）：

$$
\begin{aligned}
{}^{I_k}_{G}\dot{\bar{q}} &= \frac{1}{2}\boldsymbol{\Omega}(\boldsymbol{\omega} - \mathbf{b}_g) {}^{I_k}_{G}\bar{q} \\
\dot{\mathbf{p}}_{G/I_k} &= -[\boldsymbol{\omega} - \mathbf{b}_g]_\times \mathbf{p}_{G/I_k} + {}^{I_k}\mathbf{v} \\
{}^{G}\dot{\mathbf{g}} &= -[\boldsymbol{\omega} - \mathbf{b}_g]_\times {}^{G}\mathbf{g} - [\mathbf{v}]_\times (\mathbf{a} - \mathbf{b}_a) \\
{}^{I_k}\dot{\mathbf{v}} &= -[\boldsymbol{\omega} - \mathbf{b}_g]_\times {}^{I_k}\mathbf{v} - {}^{I_k}_{G}\mathbf{R}^\top \mathbf{g}_W + \mathbf{a} - \mathbf{b}_a \\
\dot{\mathbf{b}}_g &= \mathbf{n}_{wg}, \quad \dot{\mathbf{b}}_a = \mathbf{n}_{wa}
\end{aligned}
$$

### 5.2 数值积分

```cpp
// Propagator.cc:95-170 — CreateNewFactor() 中的状态积分
for each IMU measurement (wm, am, dt):
    w = wm - bg;   a = am - ba;
    wn = ||w||;

    // 旋转积分：Rodrigues 公式（大角度）
    if wn < small_angle:
        dR = I - dt·w× + 0.5·dt²·w×²
    else:
        dR = I - sin(wn·dt)/wn · w× + (1-cos(wn·dt))/wn² · w×²

    Rk = dR · Rk

    // 位置/速度高阶积分（含四阶系数 f1-f4）
    dp += dv·dt + RkT·(0.5·dt²·I + f1·w× + f2·w×²)·a
    dv += RkT·(dt·I + f3·w× + f4·w×²)·a
```

其中高阶系数：

$$
\begin{aligned}
f_1 &= \frac{\omega_n \Delta t \cos(\omega_n \Delta t) - \sin(\omega_n \Delta t)}{\omega_n^3} &
f_2 &= \frac{(\omega_n \Delta t)^2 - 2\cos(\omega_n \Delta t) - 2\omega_n \Delta t \sin(\omega_n \Delta t) + 2}{2\omega_n^4} \\
f_3 &= \frac{\cos(\omega_n \Delta t) - 1}{\omega_n^2} &
f_4 &= \frac{\omega_n \Delta t - \sin(\omega_n \Delta t)}{\omega_n^3}
\end{aligned}
$$

### 5.3 协方差传播

误差状态：$\tilde{\mathbf{x}} = [\tilde{\boldsymbol{\theta}}_k^\top, \tilde{\mathbf{p}}_k^\top, \tilde{\mathbf{v}}_k^\top, \tilde{\mathbf{b}}_g^\top, \tilde{\mathbf{b}}_a^\top, \tilde{\mathbf{g}}_k^\top]^\top$（18 维）。

```cpp
// Propagator.cc:111-126 — 状态转移矩阵 F (18×18)
F.block<3,3>(3,3) = -w×;          // δθ → δθ
F.block<3,3>(3,12) = -I;          // δbg → δθ
F.block<3,3>(6,3) = -RkT·v×;      // δθ → δp
F.block<3,3>(6,9) = RkT;          // δv → δp
F.block<3,3>(9,0) = -g·Rk;        // δθk → δg
F.block<3,3>(9,3) = -g·g×;        // δp → δg
F.block<3,3>(9,9) = -w×;          // δv → δg
F.block<3,3>(9,12) = -v×;         // δbg → δg
F.block<3,3>(9,15) = -I;          // δba → δg

// 噪声输入矩阵 G (18×12)
G.block<3,3>(3,0) = -I;           // ng → δθ
G.block<3,3>(9,0) = -v×;          // ng → δg
G.block<3,3>(9,6) = -I;           // na → δg
G.block<3,3>(12,3) = I;           // nwg → δbg
G.block<3,3>(15,9) = I;           // nwa → δba

// 离散化
Phi = I + dt·F                     // 一阶近似
Q = dt·G·Σ·G^T                     // 离散噪声协方差
P = Phi·P·Phi^T + Q                // 协方差更新
```

### 5.4 平方根信息因子构建

```cpp
// Propagator.cc:172-187
// 协方差 → 信息矩阵（通过 QR 求逆）
Info = qr(P.bottomRightCorner(15,15)).inverse();
Info = 0.5·(Info + Info^T);         // 对称化

// Cholesky 分解：Λ = L·L^T
Low = Info.llt().matrixL();         // L 即 R^T

// 构建观测矩阵（用 L^T 加权）
H = [Psi_bottomLeft(15×3), Psi_bottomRight(15×9), -I15];
H.applyOnTheLeft(Low.adjoint());    // H ← L^T · H
```

**原理**：传播后的残差为：

$$
J_{prop} = \|\tilde{\mathbf{x}}_{k+1} - \mathbf{f}(\tilde{\mathbf{x}}_k)\|_{\mathbf{P}^{-1}}^2
        = \|\mathbf{L}^\top(\mathbf{H} \delta\mathbf{x} - \mathbf{r})\|^2
$$

其中 $\mathbf{L}$ 来自 $\mathbf{P}^{-1} = \mathbf{L}\mathbf{L}^\top$。

### 5.5 吸收传播因子

```cpp
// Propagator.cc:190 — LocalQR()
// 将传播因子 R_prop 追加到 LocalFactor 下方
tempLocalSR = [LocalFactor; H_prop, r_prop]
// 通过 Givens 旋转恢复上三角结构
// 相当于对 [R_old; R_prop] 做 QR 分解
```

---

## 6. 视觉前端（Tracker）

### 6.1 光流跟踪

```cpp
// Tracker.cc — track() → VisualTracking()
// 使用 OpenCV KLT 光流跟踪器
// 输入：上一帧特征点 + 当前帧图像
// 输出：当前帧匹配点对
```

### 6.2 姿态预测

利用传播后的位姿预测当前相机位姿，引导特征匹配：

$$
\begin{aligned}
\mathbf{R}_{C_k G} &= \mathbf{R}_{CI} \cdot \mathbf{R}_{I_k} \cdot \mathbf{R}_{I_k G} \\
\mathbf{t}_{C_k/G} &= \mathbf{R}_{CI} \cdot \mathbf{R}_{I_k} \cdot (\mathbf{p}_{G/I_k} - \mathbf{t}_{I_k}) + \mathbf{t}_{CI}
\end{aligned}
$$

```cpp
// System.cc:309-316
Eigen::Matrix3f RkG = Rk * QuatToRot(Localx.head(4));
Eigen::Vector3f tkG = Rk * (Localx.segment(4,3) - tk);
Eigen::Matrix3f RcG = mRci * RkG;
Eigen::Vector3f tcG = mRci * tkG + mtci;
```

### 6.3 特征分类

跟踪完成后，根据跟踪长度和视差将特征分为四类（`Tracker.h:46-57`）：

| 类型 | 枚举值 | 含义 |
|------|--------|------|
| `INIT_SLAM` | 0 | 达到最大跟踪长度，用于初始化新 SLAM 地图点 |
| `POSE_ONLY_M` | 1 | 达到最大跟踪长度但深度不确定（基线×逆深度过小），降级为纯位姿约束 |
| `POSE_ONLY` | 2 | 跟踪丢失，仅用于位姿约束（丢失帧的观测不再可用） |
| `EXPLO` | 3 | 已有 3D 位置的活跃 SLAM 特征，用于持续更新 |

```cpp
// Tracker.h:123-132 — 视差计算
float Parallax(const cv::Point2f& pt0, const cv::Point2f& ptk) {
    // 将像素坐标转为方向向量
    e0 = [cos(φ0)sin(ψ0), sin(φ0), cos(φ0)cos(ψ0)]
    ek = [cos(φk)sin(ψk), sin(φk), cos(φk)cos(ψk)]
    // 视差角 = 方向向量夹角（考虑旋转补偿）
    theta = acos(ek · (Rx * e0))
    return theta * 180/π  // 返回角度制
}
```

### 6.4 特征检测与管理

```cpp
// FeatureDetector.cc — DetectWithSubPix()
// Shi-Tomasi 角点检测 + 子像素精化
// 网格均匀分布确保空间覆盖
// 避开已有特征点的邻域
```

---

## 7. 视觉更新（Updater）— 核心

### 7.1 观测模型

相机投影模型（针孔）：

$$
\mathbf{h}(\mathbf{p}_f^{C}) = \begin{bmatrix} p_{f,x}^C / p_{f,z}^C \\ p_{f,y}^C / p_{f,z}^C \end{bmatrix}, \quad
\mathbf{p}_f^C = \mathbf{R}_{CI} \cdot \mathbf{p}_f^{I_k} + \mathbf{t}_{CI}
$$

重投影误差：

$$
\mathbf{r} = \mathbf{z} - \mathbf{h}(\mathbf{p}_f^C)
$$

### 7.2 特征逆深度参数化

SLAM 特征点用逆深度 $(\phi, \psi, \rho)$ 参数化（`Updater.cc:70-142`）：

$$
\mathbf{p}_f^{C_0} = \frac{1}{\rho}
\begin{bmatrix}
\cos\phi \sin\psi \\
\sin\phi \\
\cos\phi \cos\psi
\end{bmatrix}
= \frac{1}{\rho} \mathbf{e}(\phi, \psi)
$$

其中 $C_0$ 是特征首次观测的相机坐标系。

### 7.3 三角化：Levenberg-Marquardt 优化

```cpp
// Updater.cc:70 — Updater::triangulate()
// 目标：最小化多视图重投影误差
cost = Σ ||z_i - h(Rc_i * (e(φ,ψ)/ρ) + tc_i)||²_Σinv

// LM 迭代
while (iter < maxIter) {
    (H^T R_inv H + λ·diag(H^T R_inv H)) · Δx = H^T R_inv e
    // 更新 (φ, ψ, ρ)
    if new_cost < cost: accept, λ /= 2
    else: reject, λ *= 1.5
}
```

**SLAM 点筛选**（`Updater.cc:256-266`）：

$$
\text{Threshold} = 40 \cdot \|\mathbf{t}_{C_k/C_0}\| \cdot \rho
$$

若 $\text{Threshold} < 1$，说明特征太远（或基线太短），降级为 `POSE_ONLY_M`。

### 7.4 三类观测的更新

`Updater::update()` 按顺序处理三类观测：

#### 7.4.1 EXPLO 特征更新（已有 3D 地图点）

```cpp
// Updater.cc:577-667
for each EXPLO feature:
    // 将世界坐标系 3D 点投影到当前相机帧
    p_fk = Rk·RG·p_fG + Rk·(pG - tk)
    h   = Rci·p_fk + tci
    r   = z - h(0:2)/h(2)

    // Jacobian
    Hf = d(h)/d(p_fG)         // 对 3D 点
    HG = d(h)/d(xG)           // 对全局位姿
    HP = d(h)/d(xP)           // 对外参
    Hk = d(h)/d(xk)           // 对当前相对位姿

    // Chi-square 检验
    if χ² < CHI2_THRESHOLD[2]:
        吸收观测（LocalQR）
```

**投影 Jacobian 推导**（`Updater.cc:408-453`）：

令 $\mathbf{H}_{proj} = \begin{bmatrix} 1/h_z & 0 & -h_x/h_z^2 \\ 0 & 1/h_z & -h_y/h_z^2 \end{bmatrix}$，则：

$$
\begin{aligned}
\mathbf{H}_f &= \mathbf{H}_{proj} \mathbf{R}_{CI} \mathbf{R}_k \mathbf{R}_G \\
\mathbf{H}_G &= \begin{bmatrix} \mathbf{H}_{proj}\mathbf{R}_{CI}\mathbf{R}_k[\mathbf{R}_G\mathbf{p}_{fG}^{fej}]_\times, & \mathbf{H}_{proj}\mathbf{R}_{CI}\mathbf{R}_k, & \mathbf{0}_{2\times 3} \end{bmatrix} \\
\mathbf{H}_P &= \begin{bmatrix} \mathbf{H}_{proj}[\mathbf{R}_{CI}\mathbf{p}_{fk}^{fej}]_\times, & \mathbf{H}_{proj}, & \mathbf{H}_{proj}\mathbf{R}_{CI}[\mathbf{p}_{fk}^{fej}]_\times\boldsymbol{\omega}_k - \mathbf{H}_{proj}\mathbf{R}_{CI}\mathbf{v}_k \end{bmatrix}
\end{aligned}
$$

#### 7.4.2 INIT_SLAM 特征（新地图点）

```cpp
// Updater.cc:671-838
for each INIT_SLAM feature:
    // 1. 用 LM 三角化得到 (φ, ψ, ρ)
    triangulate(nTrackLength, vRevFeatMeas, vRevRelCamPoses, φ, ψ, ρ);

    // 2. 构建完整观测因子
    //    - Hf: 对特征参数 (φ, ψ, ρ)
    //    - HP: 对外参 (qCI, pCI, td)
    //    - HW: 对滑动窗口位姿
    CreateNewFactor(featInfo, meas, poses, ..., Hf, HP, HW, r, xf);

    // 3. Givens QR 消元压缩
    //    [Hf | HP | HW | r] → QR → [Rf | R_PW | r']
    //    消除 Hf 的贡献，得到仅含位姿信息的压缩因子

    // 4. Chi-square 检验
    //    通过 → 加入 LocalFactor + 标记为活跃 SLAM 点
    //    不通过 → 标记为 BAD

    // 5. 前 3 行保留特征信息（用于后续 BA）
    Hf * mnImageNoiseSigmaInv
    // 追加到 LocalFactor 的 SLAM 特征区块
```

**滑动窗口位姿 Jacobian 构建**（`Updater.cc:296-358`）：

对于跟踪长度为 $M$ 的特征，在第 $i$ 个观测处对第 $j$ 个相对位姿 $\boldsymbol{\xi}_j$ 的 Jacobian 为：

$$
\begin{aligned}
\frac{\partial \mathbf{h}}{\partial \boldsymbol{\xi}_j} &= \mathbf{H}_{proj} \mathbf{R}_{CI} \mathbf{R}_{I_k C_{k-i}} \cdot
\begin{bmatrix}
-[\mathbf{R}_{CI}^\top \mathbf{e}/\rho + \mathbf{t}_{CI} - \mathbf{t}_{1,j}]_\times \mathbf{R}_{1,j}^\top \boldsymbol{\omega}_{k-j}, &
\rho \mathbf{R}_{1,j}^\top \mathbf{v}_{k-j}
\end{bmatrix}
\end{aligned}
$$

其中 $\mathbf{R}_{1,j}$、$\mathbf{t}_{1,j}$ 是从 $\boldsymbol{\xi}_j$ 到 $\boldsymbol{\xi}_1$ 的累积相对变换。

#### 7.4.3 POSE_ONLY 特征（纯位姿约束）

```cpp
// Updater.cc:841-956
// 与 INIT_SLAM 类似，但特征 3D 位置不进入状态
// 仅利用多视图约束更新滑动窗口位姿和外参
// 三角化结果只用于 Chi-square 检验
```

### 7.5 状态回代更新

所有观测吸收完毕后，通过回代法求解状态修正量：

```cpp
// Updater.cc:958-1044
// LocalFactor * δx = d （上三角方程组）
dLocalx = LocalFactor.rightCols(1);  // 误差向量 d
LocalFactor.leftCols(L).triangularView<Eigen::Upper>()
    .solveInPlace(dLocalx);           // 回代求解 δx
LocalFactor.rightCols(1).setZero();   // 清零误差向量

// 逐块更新状态（流形上的更新）
// 1. 丢失的 SLAM 点 3D 位置（直接加）
pf_lost += dLocalx.segment(offset, 3);

// 2. 活跃 SLAM 点 3D 位置（直接加）
pf_active += dLocalx.segment(offset, 3);

// 3. 新 SLAM 点位置（直接加）
pf_new += dLocalx.segment(offset, 3);

// 4. 全局状态 xG = (qG, pG, g)：q 用四元数乘法更新
dqG.head(3) = 0.5 * dLocalx.segment(offset, 3);
dqG(3) = sqrt(1 - ||dqG(0:2)||²);
qG ← dqG ⊗ qG;
pG += dLocalx.segment(offset+3, 3);
g  += dLocalx.segment(offset+6, 3);
g.normalize();

// 5. 外参 xP = (qCI, pCI, td)
dqP.head(3) = 0.5 * dLocalx.segment(offset, 3);
qCI ← dqP ⊗ qCI;
pCI += dLocalx.segment(offset+3, 3);
td  += dLocalx.segment(offset+6, 1);

// 6. 滑动窗口中的每个相对位姿 (qw, tw)
for each window pose:
    dqw.head(3) = 0.5 * dLocalx.segment(offset, 3);
    qw ← dqw ⊗ qw;
    tw += dLocalx.segment(offset+3, 3);

// 7. 速度 + bias（直接加）
v += dLocalx.segment(offset, 3);
bg += dLocalx.segment(offset+3, 3);
ba += dLocalx.segment(offset+6, 3);
```

### 7.6 流形上的四元数更新

```cpp
// System.cc:986-994
// 指数映射：δθ → δq
dq.head(3) = 0.5 * δθ;                       // 旋转向量 → 四元数
float n = dq.head(3).norm();
if (n > 1)
    dq = [dq(0:2)/√(1+n²), 1/√(1+n²)];       // 大旋转：归一化
else
    dq(3) = √(1-n²);                          // 小旋转
q_new = dq ⊗ q;
```

---

## 8. 平方根信息滤波核心操作

### 8.1 LocalQR：吸收新观测

```cpp
// Updater.cc:456-483 — Updater::LocalQR()
// 将新观测 H·δx = r 追加到已有信息矩阵下方
tempLocalFactor = [R_bottom | d_bottom; H | r]
// 通过 Givens 旋转从下往上消元，恢复上三角结构
for n = 0..N-1:
    for m = N+M-1 down to N:
        if temp(m,n) ≠ 0:
            Givens(temp(n,n), temp(m,n)) → 消去 temp(m,n)
```

**矩阵变换示意**：

$$
\begin{bmatrix}
\times & \times & \times & | & \times \\
0 & \times & \times & | & \times \\
0 & 0 & \times & | & \times \\
\hline
h_{11} & h_{12} & h_{13} & | & r_1 \\
h_{21} & h_{22} & h_{23} & | & r_2
\end{bmatrix}
\xrightarrow{\text{Givens QR}}
\begin{bmatrix}
\times' & \times' & \times' & | & \times' \\
0 & \times' & \times' & | & \times' \\
0 & 0 & \times' & | & \times' \\
0 & 0 & 0 & | & 0 \\
0 & 0 & 0 & | & 0
\end{bmatrix}
$$

### 8.2 ReorderQR：特征边缘化

```cpp
// Updater.cc:486-536 — Updater::ReorderQR()
// 将丢失的特征列翻转到最前面
FlipToHead(LocalFactor.block, 3);  // 每个特征占 3 列
// 然后通过 Givens 旋转恢复上三角
// 等效于舒尔补的平方根形式
```

**边缘化的平方根等价**：

传统信息形式的舒尔补：

$$
\boldsymbol{\Lambda}_{\alpha\alpha}^{'} = \boldsymbol{\Lambda}_{\alpha\alpha} - \boldsymbol{\Lambda}_{\alpha\beta}\boldsymbol{\Lambda}_{\beta\beta}^{-1}\boldsymbol{\Lambda}_{\beta\alpha}
$$

在平方根信息形式下，这等价于：
1. 将 $\beta$ 列翻转到最前
2. 通过 Givens 旋转恢复上三角
3. 取右下角 $\alpha$ 区块

### 8.3 Chi-square 检验

```cpp
// Updater.h:135-143 — Updater::chi2()
// χ² = r^T · S^{-1} · r
// 其中 S = H·U·(H·U)^T + σ²·I,  U = R^{-1}
V = H * U;         // U = NavSRinv
S = V * V^T;
S.diagonal() += σ²;  // 像素噪声
return r^T · S^{-1} · r;  // vs CHI2_THRESHOLD[DOF-1]
```

95% 置信度 Chi-square 阈值查找表（`numerics.h:156-207`）：
- DOF=2: 5.99（EXPLO 特征，2 维残差）
- DOF=M: 对应 M 维残差的阈值

---

## 9. 组合步骤（Composition）

每帧末尾执行 robocentric 状态重组合（`Updater.cc:1083-1213`）：

### 9.1 全局状态组合

```cpp
// Updater.cc:1091-1133
// 吸收新相对位姿到全局状态
xG_new = [Rk·RG; Rk·(pG-tk); Rk·g]  // 新的全局参考
// 对应 Jacobian
HG = blkdiag(Rk^T, Rk^T, Rk^T)       // qG, pG, g 的组合
Hk = -HG * Jk                          // Jk 包含 [pkG]×, -Rk, [gk]×
// 通过 ComposeQR 消元
```

### 9.2 新 SLAM 特征组合

```cpp
// Updater.cc:1139-1199
for each new SLAM feature:
    // 将特征从其参考帧 C₀ 转换到世界坐标系
    pf_G = RkG^T * (1/ρ * Rci^T * e(φ,ψ) - tci - pkG)

    // 构建特征→世界坐标的 Jacobian
    Jf = [1/ρ * RkG^T * Rci^T * Jang, -1/ρ² * RkG^T * Rci^T * e]  // (3×3)
    JG = [-[pfG_fej]× * RkG^T, -RkG^T]                              // 对全局位姿
    JP = [-RkG^T * Rci^T * [e/ρ - tci]×, -RkG^T * Rci^T, 0]        // 对外参

    // 通过 QR 分解求 Jf 的逆
    qr(Jf);  Hf = qr.inverse();  HG = -Hf*JG;  HP = -Hf*JP;

    // 更新 LocalFactor
    LocalFactor.block = LocalFactor.block * Hf + ...
    ComposeQR(offset, 3*nNewFeatures, LocalFactor);
```

### 9.3 边缘化丢失的特征

```cpp
// Updater.cc:1202-1212
if (nLostActiveFeatures > 0) {
    // 裁剪 LocalFactor：移除已边缘化的特征块
    LocalFactor = LocalFactor.bottomRightCorner(nDimOfSR, nDimOfSR+1);
    // 标记特征为已边缘化
    for each lost id: mFeatures.at(id)->Marginalized();
}
```

---

## 10. 在线时空标定

R-VIO2 同时在线估计相机-IMU 的外参和时间偏移。

### 10.1 外参

外参 $(\mathbf{R}_{CI}, \mathbf{t}_{CI})$ 作为状态变量出现在 `Localx` 中：

```cpp
// System.cc:234-235
Localx.segment(10,4) = RotToQuat(mRci);  // qCI
Localx.segment(14,3) = mtci;              // pCI (在 {I} 坐标系中表示)
```

在每个观测 Jacobian 中都包含对外参的导数（`Updater.cc:354-355`）：

$$
\begin{aligned}
\frac{\partial \mathbf{h}}{\partial \boldsymbol{\theta}_{CI}} &=
\mathbf{H}_{proj} \left( [\mathbf{R}_{CI}\mathbf{p}_f^{C_0}]_\times - \rho[\mathbf{R}_{CI}\mathbf{t}_{CI}]_\times \right) (\mathbf{I} - \mathbf{R}_{CI}) \\
\frac{\partial \mathbf{h}}{\partial \mathbf{t}_{CI}} &=
\mathbf{H}_{proj} \cdot \rho \cdot (\mathbf{I} - \mathbf{R}_{CI})
\end{aligned}
$$

### 10.2 时间偏移

时间偏移 $t_d$ 补偿 IMU 和相机之间的时间差（`System.cc:84-86`）：

```cpp
// 测量获取时使用时间偏移做对齐
mpInputBuffer->GetMeasurements(mnCamTimeOffset, pMeasurements);
```

外参 Jacobian 中包含时间偏移项（`Updater.cc:334`）：

$$
\frac{\partial \mathbf{h}}{\partial t_d} = \mathbf{H}_{proj} \left( \mathbf{R}_{CI}[\mathbf{p}_{fk}^{fej}]_\times \boldsymbol{\omega}_k - \mathbf{R}_{CI} \mathbf{v}_k \right)
$$

初始先验（`System.cc:255`）：
```cpp
LocalFactor(15,15) = mbUseGroundTruthCalib ? 10 * mnImuRate : mnCamRate;
```

---

## 11. 关键数据结构

### 11.1 Feature（特征点）

```cpp
// Feature.h:34-84
class Feature {
    int mnFeatureId;              // 全局唯一 ID
    int mnRootImageId;            // 首次观测的图像帧 ID
    bool mbIsInited;              // 是否已初始化 3D 位置
    bool mbIsMarginalized;        // 是否已边缘化
    Eigen::Vector3f mPosition;    // (φ,ψ,ρ) 或世界坐标 pf_G
    Eigen::Vector3f mFejPosition; // FEJ（First-Estimate Jacobian）位置
};
```

**FEJ（First-Estimate Jacobian）**：在特征首次被三角化时记录其线性化点，后续 Jacobian 始终在此点计算，保证估计器的一致性。

### 11.2 滑动窗口管理

```cpp
// System.cc:326-332
// 保存局部速度（用于后续帧的 Jacobian 计算）
mqLocalw.push_back(w);   // 角速度
mqLocalv.push_back(v);   // 线速度
if (nImageId > mnLocalWindowSize) {
    mqLocalw.pop_front();  // 窗口满后移除最旧帧
    mqLocalv.pop_front();
}
```

---

## 12. 算法参数配置

关键配置参数（来自 YAML 配置文件）：

| 参数 | 含义 | 典型值 |
|------|------|--------|
| `Tracker.nMaxTrackingLength` | 最大跟踪长度 | 决定滑动窗口大小 |
| `Tracker.nMaxSlamPoints` | 最大 SLAM 点数 | 0 = VIO 模式, >0 = SLAM 模式 |
| `IMU.sigma_g` | 陀螺仪噪声标准差 | IMU 数据手册 |
| `IMU.sigma_a` | 加速度计噪声标准差 | IMU 数据手册 |
| `IMU.sigma_wg` | 陀螺仪随机游走 | IMU 数据手册 |
| `IMU.sigma_wa` | 加速度计随机游走 | IMU 数据手册 |
| `Camera.sigma_px/y` | 像素噪声标准差 | 1.0 px |
| `INI.nAngleThrd` | 静止检测角度阈值 | ~0.5° |
| `INI.nLengthThrd` | 静止检测位移阈值 | ~0.01 m |

---

## 13. VIO vs SLAM 模式对比

| 特性 | VIO 模式 | SLAM 模式 |
|------|:--------:|:---------:|
| 地图点 | 无持久地图点 | 维护稀疏 SLAM 地图点 |
| 状态维度 | 较小（仅滑动窗口位姿） | 较大（额外包含 3D 点） |
| 漂移 | 纯里程计，存在漂移 | 回环可减少漂移 |
| 对应论文 | RA-L 2022 | T-RO 2024 |
| 配置 | `nMaxSlamPoints = 0` | `nMaxSlamPoints > 0` |

---

## 14. 算法伪代码

```
输入: IMU 数据流, 相机图像流
输出: 6-DOF 全局位姿, 在线标定参数

初始化:
  等待静止 → 估计 bg, ba, g
  构建先验 LocalFactor

循环每帧:
  1. 时间对齐 IMU + 图像
  2. IMU 传播:
     for each IMU meas:
       状态积分（Rodrigues + 高阶系数）
       协方差传播（F, G, Phi, Q）
     构建平方根信息因子
     LocalQR 吸收传播因子
  3. 预测相机位姿: (RcG, tcG)
  4. 视觉跟踪:
     光流跟踪 + RANSAC
     特征分类: EXPLO / INIT_SLAM / POSE_ONLY
  5. 视觉更新:
     for each 特征类型:
       构建观测 Jacobian
       Chi-square 检验
       LocalQR 吸收观测
     回代求解 δx
     流形上更新状态
  6. Composition:
     全局状态重组合
     新 SLAM 特征世界坐标转换
     边缘化丢失特征
  7. 发布位姿 + 可视化
```

---

## 参考文献

1. Huai, Z., & Huang, G. (2022). Square-Root Robocentric Visual-Inertial Odometry with Online Spatiotemporal Calibration. *IEEE Robotics and Automation Letters*, 7(4), 9961–9968.
2. Huai, Z., & Huang, G. (2024). A Consistent Parallel Estimation Framework for Visual-Inertial SLAM. *IEEE Transactions on Robotics*, 40, 3734–3755.
3. Huai, Z., & Huang, G. (2019). Robocentric Visual-Inertial Odometry. *International Journal of Robotics Research*, 38(8).

