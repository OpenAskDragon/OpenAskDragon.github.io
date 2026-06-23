---
title: "项目文档"
date: 2026-06-22
tags: []
gitalk: false
---

# 项目文档

这里存放各项目的设计文档、API 说明和开发记录。

---

## 📂 文档列表

### [R-VIO2 算法详解](./rvio2_algorithm)

基于单目相机和 IMU 的平方根机器人中心视觉-惯性里程计。详解 robocentric 状态表示、平方根信息滤波、IMU 传播、视觉前端跟踪、QR 消元更新、在线时空标定等核心算法。

> 论文：Zheng Huai & Guoquan Huang, *IEEE RA-L 2022 / T-RO 2024*

### [3DGS：核心原理、技术路线与代码对应](./3dgs_algorithm)

围绕 3D Gaussian Splatting 的场景表示、可微 splatting、训练优化、densification、几何一致性、视觉定位与 Nerfstudio 插件实现，整理公式、技术路线和代码对应关系。

### [StreetCrafter 算法逻辑与工程流程说明](./algorithm_logic_flow.zh)

面向自动驾驶街景新视角合成的工程文档，梳理 LiDAR 条件视频扩散、动态 3D Gaussian 蒸馏、真实/新视角损失、SDS 采样初始化，以及核心公式与代码变量的对应关系。

