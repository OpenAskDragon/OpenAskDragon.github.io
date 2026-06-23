---
title: "3DGS：核心原理、技术路线与代码对应"
date: 2026-06-23
tags: [3DGS, Gaussian-Splatting, RaDe-GS, gsplat, SplatLoc, GSplatLoc, Nerfstudio]
gitalk: false
---

# 3DGS：核心原理、技术路线与代码对应

> **主题范围**：3D Gaussian Splatting 及其在渲染、深度几何重建、视觉定位和 Nerfstudio 插件生态中的几种实现路线。  
> **涉及项目**：`gaussian-splatting`、`RaDe-GS`、`gsplat`、`SplatLoc`、`gsplatloc`、`nerfstudio`、`splatfacto-w`。  

---

## 1. 算法概述

3D Gaussian Splatting（3DGS）是一类显式辐射场表示方法。它将场景表示为一组可优化的三维高斯 primitive，通过可微 rasterization 直接渲染图像。与 NeRF 类方法相比，3DGS 不需要对每条光线执行密集 MLP 查询，而是将三维高斯投影为屏幕空间椭圆 splat，并按深度顺序进行 alpha compositing。

标准 3DGS 的核心要素包括：

1. **显式场景表示**：每个 primitive 是一个带颜色、透明度和各向异性协方差的 3D Gaussian；
2. **可微 splatting rasterizer**：将三维高斯投影到二维图像平面，计算颜色、深度和梯度；
3. **联合优化**：优化位置、尺度、旋转、不透明度和球谐颜色系数；
4. **自适应密度控制**：根据屏幕空间梯度进行 clone、split、prune 和 opacity reset；
5. **扩展属性场**：在定位或语义任务中，可将 descriptor、feature map、marker score、depth、normal 等属性绑定到 Gaussian。

不同项目的侧重点不同：

| 项目 | 技术定位 | 关键扩展 |
|---|---|---|
| `gaussian-splatting` | 原始 3DGS 训练与渲染基线 | RGB 重建、SH 颜色、densification、SIBR viewer |
| `RaDe-GS` | 深度/法线/几何增强 3DGS | expected/median depth、normal consistency、multi-view NCC、mesh extraction |
| `gsplat` | 高性能 Gaussian rasterization 库 | batched rasterization、packed mode、2DGS/3DGUT/LiDAR render mode |
| `SplatLoc` | 3DGS 视觉定位系统 | 3D feature decoder、marker probability、2D-3D matching、PnP |
| `gsplatloc` | keypoint descriptor grounded 3DGS | XFeat descriptor、Gaussian feature field、PnP-RANSAC、warp refinement |
| `nerfstudio` | 神经渲染框架 | 数据处理、训练 pipeline、viewer、插件注册 |
| `splatfacto-w` | Nerfstudio in-the-wild 3DGS 插件 | transient/appearance/background 建模 |

---

## 2. 3D Gaussian 表示

### 2.1 单个 Gaussian 的参数

一个 3D Gaussian 可写为：

$$
G_i(\mathbf{x}) = \exp\left(-\frac{1}{2}(\mathbf{x}-\boldsymbol{\mu}_i)^\top
\boldsymbol{\Sigma}_i^{-1}
(\mathbf{x}-\boldsymbol{\mu}_i)\right)
$$

其中：

- $\boldsymbol{\mu}_i \in \mathbb{R}^3$：Gaussian 中心；
- $\boldsymbol{\Sigma}_i \in \mathbb{R}^{3\times3}$：三维协方差；
- $\alpha_i \in (0,1)$：不透明度；
- $\mathbf{c}_i(\mathbf{d})$：视角相关颜色，通常用 spherical harmonics 表示；
- 扩展系统中还可能含有 descriptor、semantic feature、depth confidence、marker score 等属性。

在代码中，原始 3DGS 的参数集中在 `gaussian-splatting/scene/gaussian_model.py`：

```python
class GaussianModel:
    self._xyz           # Gaussian center, μ
    self._features_dc   # SH DC color
    self._features_rest # higher-order SH coefficients
    self._scaling       # log scale
    self._rotation      # quaternion
    self._opacity       # inverse-sigmoid opacity parameter
```

### 2.2 协方差参数化

3DGS 不直接优化完整协方差矩阵，而是使用尺度和旋转构造：

$$
\boldsymbol{\Sigma}_i = \mathbf{R}_i \mathbf{S}_i \mathbf{S}_i^\top \mathbf{R}_i^\top
$$

其中 $\mathbf{S}_i = \operatorname{diag}(s_x, s_y, s_z)$，$\mathbf{R}_i$ 来自四元数。

代码对应：

```python
# gaussian-splatting/scene/gaussian_model.py
def build_covariance_from_scaling_rotation(scaling, scaling_modifier, rotation):
    L = build_scaling_rotation(scaling_modifier * scaling, rotation)
    actual_covariance = L @ L.transpose(1, 2)
    symm = strip_symmetric(actual_covariance)
    return symm

self.scaling_activation = torch.exp
self.rotation_activation = torch.nn.functional.normalize
```

这里优化变量是 unconstrained 的 `_scaling` 和 `_rotation`。`torch.exp` 保证尺度为正，`normalize` 保证四元数单位化。

### 2.3 PLY 中的字段

训练结果通常保存为 PLY，每个顶点对应一个 Gaussian：

```text
x, y, z
f_dc_*
f_rest_*
opacity
scale_*
rot_*
```

代码对应：

```python
# gaussian-splatting/scene/gaussian_model.py
def construct_list_of_attributes(self):
    l = ['x', 'y', 'z', 'nx', 'ny', 'nz']
    ...
    l.append('opacity')
    for i in range(self._scaling.shape[1]):
        l.append('scale_{}'.format(i))
    for i in range(self._rotation.shape[1]):
        l.append('rot_{}'.format(i))
```

---

## 3. 可微 Gaussian Splatting 渲染

### 3.1 投影与屏幕空间 footprint

三维点经相机外参和内参投影：

$$
\mathbf{x}_c = \mathbf{R}_{cw}\boldsymbol{\mu} + \mathbf{t}_{cw}, \quad
\mathbf{u} = \pi(\mathbf{x}_c)
$$

协方差通过投影 Jacobian $\mathbf{J}$ 近似传播到屏幕空间：

$$
\boldsymbol{\Sigma}_{2D} = \mathbf{J}\mathbf{W}\boldsymbol{\Sigma}_{3D}\mathbf{W}^\top\mathbf{J}^\top
$$

其中 $\mathbf{W}$ 是 world-to-camera 旋转。屏幕上每个 Gaussian 成为一个椭圆 splat，rasterizer 按 tile 找到可能覆盖的像素。

原始实现的渲染入口：

```python
# gaussian-splatting/gaussian_renderer/__init__.py
def render(viewpoint_camera, pc, pipe, bg_color, scaling_modifier=1.0, ...):
    screenspace_points = torch.zeros_like(
        pc.get_xyz, requires_grad=True, device="cuda"
    )

    raster_settings = GaussianRasterizationSettings(
        image_height=int(viewpoint_camera.image_height),
        image_width=int(viewpoint_camera.image_width),
        tanfovx=math.tan(viewpoint_camera.FoVx * 0.5),
        tanfovy=math.tan(viewpoint_camera.FoVy * 0.5),
        viewmatrix=viewpoint_camera.world_view_transform,
        projmatrix=viewpoint_camera.full_proj_transform,
        sh_degree=pc.active_sh_degree,
        campos=viewpoint_camera.camera_center,
    )
```

### 3.2 颜色模型：Spherical Harmonics

视角相关颜色用球谐函数表示：

$$
\mathbf{c}_i(\mathbf{d}) =
\sum_{\ell=0}^{L}\sum_{m=-\ell}^{\ell}
\mathbf{k}_{i,\ell m}Y_{\ell m}(\mathbf{d})
$$

其中 $\mathbf{d}$ 是从相机到 Gaussian 的方向。代码中可以选择在 Python 侧计算 SH，也可以让 rasterizer 内部计算：

```python
# gaussian-splatting/gaussian_renderer/__init__.py
if pipe.convert_SHs_python:
    shs_view = pc.get_features.transpose(1, 2).view(-1, 3, (pc.max_sh_degree+1)**2)
    dir_pp = pc.get_xyz - viewpoint_camera.camera_center.repeat(pc.get_features.shape[0], 1)
    dir_pp_normalized = dir_pp / dir_pp.norm(dim=1, keepdim=True)
    sh2rgb = eval_sh(pc.active_sh_degree, shs_view, dir_pp_normalized)
    colors_precomp = torch.clamp_min(sh2rgb + 0.5, 0.0)
else:
    shs = pc.get_features
```

### 3.3 Alpha compositing

对同一像素，按照深度从近到远合成：

$$
\mathbf{C} = \sum_{i=1}^{N} T_i \alpha_i \mathbf{c}_i,\quad
T_i = \prod_{j=1}^{i-1}(1-\alpha_j)
$$

其中 $T_i$ 是第 $i$ 个 Gaussian 前方的透射率。这个公式是 3DGS 可微 rasterization 的核心，与 NeRF 的体渲染形式相似，但积分对象从连续 ray samples 变成显式 splat primitive。

---

## 4. 训练目标与优化循环

### 4.1 RGB 重建损失

原始 3DGS 的基本损失是 L1 与 DSSIM 加权：

$$
\mathcal{L}_{rgb} =
(1-\lambda)\|\hat{\mathbf{I}}-\mathbf{I}\|_1
+ \lambda(1-\operatorname{SSIM}(\hat{\mathbf{I}},\mathbf{I}))
$$

代码对应：

```python
# gaussian-splatting/train.py
Ll1 = l1_loss(image, gt_image)
ssim_value = ssim(image, gt_image)
loss = (1.0 - opt.lambda_dssim) * Ll1 + opt.lambda_dssim * (1.0 - ssim_value)
loss.backward()
```

### 4.2 主训练循环

```python
# gaussian-splatting/train.py
for iteration in range(first_iter, opt.iterations + 1):
    gaussians.update_learning_rate(iteration)

    if iteration % 1000 == 0:
        gaussians.oneupSHdegree()

    viewpoint_cam = random_training_camera()
    render_pkg = render(viewpoint_cam, gaussians, pipe, bg)

    image = render_pkg["render"]
    loss = rgb_loss(image, viewpoint_cam.original_image)
    loss.backward()

    if iteration < opt.densify_until_iter:
        gaussians.add_densification_stats(...)
        if iteration > opt.densify_from_iter and iteration % opt.densification_interval == 0:
            gaussians.densify_and_prune(...)

    gaussians.optimizer.step()
    gaussians.optimizer.zero_grad()
```

### 4.3 优化变量与学习率组

```python
# gaussian-splatting/scene/gaussian_model.py
l = [
    {'params': [self._xyz], 'lr': position_lr, "name": "xyz"},
    {'params': [self._features_dc], 'lr': feature_lr, "name": "f_dc"},
    {'params': [self._features_rest], 'lr': feature_lr / 20.0, "name": "f_rest"},
    {'params': [self._opacity], 'lr': opacity_lr, "name": "opacity"},
    {'params': [self._scaling], 'lr': scaling_lr, "name": "scaling"},
    {'params': [self._rotation], 'lr': rotation_lr, "name": "rotation"},
]
```

位置学习率使用指数调度：

$$
\eta(t) = \eta_{init}\left(\frac{\eta_{final}}{\eta_{init}}\right)^{t/T}
$$

代码通过 `get_expon_lr_func(...)` 构造调度函数，并在每次迭代调用 `update_learning_rate(iteration)`。

---

## 5. Densification 与 Pruning

### 5.1 屏幕空间梯度统计

3DGS 使用屏幕空间位置梯度判断哪些 Gaussian 对图像误差敏感：

$$
g_i = \left\|\frac{\partial\mathcal{L}}{\partial \boldsymbol{\mu}_{i,2D}}\right\|_2
$$

代码对应：

```python
# gaussian-splatting/scene/gaussian_model.py
def add_densification_stats(self, viewspace_point_tensor, update_filter):
    self.xyz_gradient_accum[update_filter] += torch.norm(
        viewspace_point_tensor.grad[update_filter, :2], dim=-1, keepdim=True
    )
    self.denom[update_filter] += 1
```

平均梯度：

$$
\bar{g}_i = \frac{\sum_t g_{i,t}}{n_i}
$$

### 5.2 Clone 与 Split

当梯度高且 Gaussian 较小，使用 clone：

```python
# gaussian-splatting/scene/gaussian_model.py
selected = norm(grads) >= grad_threshold
selected &= max(get_scaling) <= percent_dense * scene_extent
new_xyz = self._xyz[selected]
```

当梯度高且 Gaussian 较大，使用 split：

```python
selected = padded_grad >= grad_threshold
selected &= max(get_scaling) > percent_dense * scene_extent

samples = torch.normal(mean=0, std=get_scaling[selected])
new_xyz = R @ samples + get_xyz[selected]
new_scaling = log(get_scaling[selected] / (0.8 * N))
```

对应数学形式：

$$
\boldsymbol{\mu}' = \boldsymbol{\mu} + \mathbf{R}\boldsymbol{\epsilon},
\quad
\boldsymbol{\epsilon}\sim\mathcal{N}(\mathbf{0},\operatorname{diag}(\mathbf{s})^2)
$$

### 5.3 Pruning

低透明度、屏幕空间过大或世界空间过大的 Gaussian 被删除：

```python
# gaussian-splatting/scene/gaussian_model.py
prune_mask = (self.get_opacity < min_opacity).squeeze()
if max_screen_size:
    big_points_vs = self.max_radii2D > max_screen_size
    big_points_ws = self.get_scaling.max(dim=1).values > 0.1 * extent
    prune_mask = prune_mask | big_points_vs | big_points_ws
self.prune_points(prune_mask)
```

---

## 6. 原始 3DGS 项目：训练、渲染、评估

### 6.1 模块组成

| 模块 | 文件 | 职责 |
|---|---|---|
| 模型表示 | `gaussian-splatting/scene/gaussian_model.py` | Gaussian 参数、PLY 读写、densification |
| 场景管理 | `gaussian-splatting/scene/__init__.py` | 加载相机、点云和训练/测试视角 |
| 数据读取 | `gaussian-splatting/scene/dataset_readers.py` | COLMAP、NeRF synthetic 数据解析 |
| 渲染器 | `gaussian-splatting/gaussian_renderer/__init__.py` | 调用 CUDA rasterizer |
| 训练入口 | `gaussian-splatting/train.py` | 随机视角训练、loss、density control |
| 渲染入口 | `gaussian-splatting/render.py` | 输出 train/test 渲染图 |
| 评价入口 | `gaussian-splatting/metrics.py` | PSNR、SSIM、LPIPS |

### 6.2 数据模型

COLMAP 数据读取后被转换为：

```python
CameraInfo(uid, R, T, FovY, FovX, image, image_path, image_name, width, height)
SceneInfo(point_cloud, train_cameras, test_cameras, nerf_normalization, ply_path)
```

Gaussian 初始化由 `create_from_pcd(...)` 完成：

```python
fused_point_cloud = torch.tensor(np.asarray(pcd.points)).float().cuda()
fused_color = RGB2SH(torch.tensor(np.asarray(pcd.colors)).float().cuda())
dist2 = distCUDA2(points)
scales = torch.log(torch.sqrt(dist2))[..., None].repeat(1, 3)
opacities = inverse_sigmoid(0.1 * ones)
```

初始化尺度来自点云最近邻距离，含义是让初始 Gaussian 大小与稀疏点云局部间距一致。

---

## 7. RaDe-GS：深度与几何一致性

RaDe-GS 在 3DGS 基础上强化几何可解释性。其渲染器不仅返回 RGB，还返回 expected depth、median depth、alpha mask 和 normal。

### 7.1 渲染输出扩展

```python
# RaDe-GS/gaussian_renderer/__init__.py
rendered_image, radii, expected_depth, median_depth, alpha, normal = rasterizer(...)

return {
    "render": rendered_image,
    "mask": rendered_alpha,
    "expected_depth": rendered_expected_depth,
    "median_depth": rendered_median_depth,
    "normal": rendered_normal,
}
```

expected depth 可理解为 alpha 权重下的期望深度：

$$
D_{exp}(\mathbf{u}) =
\frac{\sum_i T_i\alpha_i z_i}{\sum_i T_i\alpha_i+\epsilon}
$$

median depth 更接近累积透明度达到一定阈值时的表面深度，通常对厚重 splat 或遮挡边界更稳健。

### 7.2 Depth-to-normal 一致性

给定深度图 $D(u,v)$ 和相机射线方向 $\mathbf{r}(u,v)$，可反投影得到三维点：

$$
\mathbf{P}(u,v) = D(u,v)\mathbf{r}(u,v)
$$

局部法线由相邻点叉乘得到：

$$
\mathbf{a} = \mathbf{P}_{u+1,v}-\mathbf{P}_{u-1,v}, \quad
\mathbf{b} = \mathbf{P}_{u,v+1}-\mathbf{P}_{u,v-1}
$$

$$
\mathbf{n}_D =
\frac{\mathbf{a} \times \mathbf{b}}
{\left\|\mathbf{a} \times \mathbf{b}\right\|}
$$

代码对应：

```python
# RaDe-GS/utils/graphics_utils.py
normal_map = torch.nn.functional.normalize(torch.cross(dy, dx, dim=1), dim=1)
```

训练损失：

$$
\mathcal{L}_{normal} = 1 - \langle \mathbf{n}_{render}, \mathbf{n}_{depth}\rangle
$$

```python
# RaDe-GS/train.py
normal_error_map = 1 - torch.linalg.vecdot(rendered_normal, depth_normal, dim=0)
depth_normal_loss = normal_error_map.mean()
```

### 7.3 多视图 NCC 与几何误差

RaDe-GS 使用 patch warping 做多视图光度一致性。设参考视角 patch 为 $P_r$，邻近视角 warp 后 patch 为 $P_s$，NCC 为：

$$
\operatorname{NCC}(P_r,P_s)=
\frac{\sum (P_r-\bar{P}_r)(P_s-\bar{P}_s)}
{\sqrt{\sum(P_r-\bar{P}_r)^2}\sqrt{\sum(P_s-\bar{P}_s)^2}}
$$

损失：

$$
\mathcal{L}_{ncc} = \operatorname{clamp}(1-\operatorname{NCC},0,2)
$$

代码对应：

```python
# RaDe-GS/utils/loss_utils.py
cc, valid_mask = warp_patch_ncc.warp_patch_ncc(
    depth_select, normal_select, ...
)
ncc = torch.clamp(1 - cc, 0.0, 2.0)
ncc_loss = ncc[ncc_mask].mean()
```

训练总损失：

$$
\mathcal{L} =
\mathcal{L}_{rgb}
+ \lambda_n\mathcal{L}_{normal}
+ \lambda_{ncc}\mathcal{L}_{ncc}
+ \lambda_g\mathcal{L}_{geo}
$$

```python
# RaDe-GS/train.py
loss = (
    rgb_loss
    + opt.lambda_depth_normal * depth_normal_loss
    + opt.lambda_multi_view_ncc * ncc_loss
    + opt.lambda_multi_view_geo * geo_loss
)
```

### 7.4 Mesh extraction

RaDe-GS 提供 `mesh_extract.py`、`mesh_extract_tnt.py` 和 `mesh_extract_tetrahedra.py`。其中 `integrate(...)` 和 `sample_depth(...)` 允许在空间点或 tetrahedra 顶点处评估 Gaussian 的 alpha/depth 信息，用于从 splat 表示转为 mesh 表示。

---

## 8. gsplat：高性能 Rasterization API

`gsplat` 将 Gaussian rasterization 抽象为 Python API，输入 tensor 直接对应 Gaussian 参数：

```python
# gsplat/gsplat/rendering.py
def rasterization(
    means,      # [..., N, 3]
    quats,      # [..., N, 4]
    scales,     # [..., N, 3]
    opacities,  # [..., N]
    colors,     # [..., N, D] or SH coefficients
    viewmats,
    Ks,
    width,
    height,
    render_mode="RGB",
    packed=True,
    sparse_grad=False,
    camera_model="pinhole",
):
    ...
```

### 8.1 Packed mode

`packed=True` 时，中间可见 Gaussian-camera pair 被压缩为 sparse/packed 结构，降低大场景和多相机场景的内存占用：

```text
non-packed: [batch, camera, gaussian, ...]
packed:     [nnz, ...]
```

这对应稀疏可见性假设：任一相机实际只看到全部 Gaussian 的一部分。

### 8.2 Render mode

`render_mode` 控制输出通道：

| 模式 | 含义 |
|---|---|
| `RGB` | 颜色渲染 |
| `D` / `ED` | 深度或期望深度 |
| `RGB+D` / `RGB+ED` | 同时输出颜色和深度 |
| LiDAR/eval3d 相关模式 | 支持 3D ray / spinning LiDAR 渲染 |

这使 `gsplat` 不只是 viewer 后端，也可作为训练、评估、定位或传感器仿真的基础 rasterizer。

---

## 9. SplatLoc：3DGS 视觉定位

SplatLoc 将 3DGS 从“图像重建表示”扩展为“定位用 3D feature field”。核心思想是：为 Gaussian 学习 3D descriptor，使 2D 图像特征可以与 3D Gaussian 匹配，从而恢复相机位姿。

### 9.1 Feature decoder

3D 点 $\mathbf{x}$ 经位置编码后输入 MLP：

$$
\mathbf{f}_{3D}(\mathbf{x}) =
\operatorname{MLP}(\gamma(\mathbf{x}))
$$

其中 $\gamma$ 可以是 HashGrid、frequency encoding 或 identity encoding。

代码对应：

```python
# SplatLoc/models/decoders.py
class FeatureDecoder(nn.Module):
    ...
```

decoder 训练使用 cosine loss：

$$
\mathcal{L}_{cos} = 1 -
\frac{\mathbf{f}_{pred}^\top\mathbf{f}_{gt}}
{\|\mathbf{f}_{pred}\|\|\mathbf{f}_{gt}\|}
$$

```python
# SplatLoc/train_decoder.py
def cos_loss(network_output, gt):
    return 1 - cosine_similarity(network_output, gt)
```

### 9.2 Gaussian 训练中的 descriptor 与 marker

SplatLoc 的 Gaussian renderer 返回：

```python
render_pkg["render"]      # RGB
render_pkg["kp_prob"]     # keypoint probability / marker
render_pkg["depth"]       # depth
render_pkg["opacity"]     # opacity
```

训练中包含 marker loss、descriptor loss、RGB/depth mapping loss 和 primitive regularization：

```python
# SplatLoc/train_gaussians.py
loss_mapping += get_loss_marker(config, marker, viewpoint.kp_score)
loss_mapping += get_loss_descriptor(config, pred_descriptor, gt_descriptor)
loss_mapping += get_loss_mapping(config, image, depth, viewpoint, opacity)
```

marker 约束用于筛选更适合作为 3D landmark 的 Gaussian。

### 9.3 2D-3D matching 与 PnP

SplatLoc 在查询图像中提取 2D keypoint descriptor，在 3D Gaussian 中查询 descriptor，得到 2D-3D 对应：

```python
# SplatLoc/utils/match_utils.py
similarity = torch.matmul(descriptors1.t(), descriptors2)
matches = linear_sum_assignment(-similarity)
```

位姿由 PnP 求解：

```python
# SplatLoc/test.py
def solve_pose(kp_2d, kp_3d, intrinsics):
    cv2.solvePnPRansac(kp_3d, kp_2d, intrinsics, ...)
```

数学形式：

$$
\min_{\mathbf{R},\mathbf{t}}
\sum_i
\left\|
\mathbf{u}_i -
\pi\left(\mathbf{K}(\mathbf{R}\mathbf{X}_i+\mathbf{t})\right)
\right\|^2
$$

---

## 10. GSplatLoc：关键点描述子驱动的定位

GSplatLoc 同样将 descriptor grounding 到 3DGS，但更强调 XFeat sparse keypoint 与 Gaussian feature 的直接匹配。

### 10.1 2D-3D 对应搜索

```python
# gsplatloc/utils/loc_utils.py
image_features = F.normalize(image_features, p=2, dim=1)
gaussian_feat = F.normalize(gaussian_feat, p=2, dim=1)
similarity = torch.mm(image_features, chunk.t())
```

使用 cosine similarity：

$$
s_{ij} =
\frac{\mathbf{f}^{2D}_i \cdot \mathbf{f}^{3D}_j}
{\|\mathbf{f}^{2D}_i\|\|\mathbf{f}^{3D}_j\|}
$$

取最相似的 Gaussian 作为 2D keypoint 的 3D 对应。

### 10.2 PnP-RANSAC 初值

```python
# gsplatloc/loc_inference.py
_, R, t, _ = cv2.solvePnPRansac(
    matched_3d,
    matched_2d,
    K,
    distCoeffs=None,
    iterationsCount=args.ransac_iters,
)
```

RANSAC 处理错误匹配，输出初始位姿。

### 10.3 Differentiable warp refinement

PnP 后，GSplatLoc 渲染当前估计位姿下的图像和深度，再通过可微 warping 优化位姿：

```python
# gsplatloc/warping/warping_loss.py
warp = pose @ from_cam_tensor_to_w2c(torch.cat([quat_opt, t_opt], dim=0)).inverse()
warped_image = differentiable_warp(rendered_image, depth, warp, K)
loss = F.mse_loss(warped_image, query_image)
```

优化目标：

$$
\min_{\Delta\mathbf{q},\Delta\mathbf{t}}
\left\|
\operatorname{warp}
(\hat{\mathbf{I}}, \hat{\mathbf{D}}, \Delta\mathbf{T})
- \mathbf{I}_{query}
\right\|_2^2
$$

---

## 11. Nerfstudio 与 Splatfacto-W

Nerfstudio 提供统一的训练框架、dataparser、viewer 和 CLI。`splatfacto-w` 是基于 Nerfstudio 的 in-the-wild Gaussian Splatting 插件。

### 11.1 插件注册

```toml
# splatfacto-w/pyproject.toml
[project.entry-points.'nerfstudio.method_configs']
splatfactow = 'splatfactow.splatfactow_config:splatfactow_config'
splatfactow_light = 'splatfactow.splatfactow_config:splatfactow_light_config'

[project.entry-points.'nerfstudio.dataparser_configs']
splatfactow_dataparser = 'splatfactow.nerfw_dataparser:splatfactow_dataparser'
```

这使方法可以通过 `ns-train splatfacto-w ...` 调用。

### 11.2 In-the-wild 建模

野外图像存在曝光变化、动态物体、天空背景和 transient object。Splatfacto-W 的扩展包括：

- appearance modeling：为不同图像建模外观差异；
- background modeling：处理远景和天空区域；
- alpha loss：约束 sky 等区域不被前景 Gaussian 占据；
- robust mask：降低 transient object 对静态场景的影响。

这些设计与标准 3DGS 的主要区别在于：训练目标不再假设所有像素都来自同一个静态、光照一致的场景。

---

## 12. 技术路线对比

| 维度 | 原始 3DGS | RaDe-GS | SplatLoc | GSplatLoc | Splatfacto-W |
|---|---|---|---|---|---|
| 主要目标 | novel view synthesis | 几何/mesh/深度一致性 | AR 视觉定位 | 关键点定位 | 野外图像重建 |
| 优化对象 | RGB + SH + opacity + geometry | RGB + depth + normal + NCC | RGB + descriptor + marker | RGB/feature + pose | RGB + appearance/transient |
| 核心输出 | Gaussian PLY、rendered images | Gaussian + depth/normal/mesh | camera pose、landmark selection | refined camera pose | robust Gaussian scene |
| 关键损失 | L1 + DSSIM | RGB + normal + NCC + geo | marker/descriptor/rendering | feature matching + warp | robust photometric losses |
| 典型入口 | `train.py` | `train.py`, `mesh_extract.py` | `train_decoder.py`, `test.py` | `train.py`, `loc_inference.py` | `ns-train splatfacto-w` |

---

## 13. 算法伪代码

```text
输入:
  多视角图像、相机参数、初始点云或 SfM 点云

初始化:
  1. 从点云创建 Gaussian centers μ
  2. 用最近邻距离初始化 scale
  3. 用 RGB 初始化 SH DC
  4. 初始化 opacity 和 rotation

训练循环:
  for iteration = 1..T:
    1. 随机选择训练相机
    2. 投影 3D Gaussian 到屏幕空间
    3. rasterize 得到 RGB / depth / normal / feature
    4. 计算损失:
       - 原始 3DGS: L1 + DSSIM
       - RaDe-GS: RGB + normal + NCC + geometry
       - 定位扩展: descriptor + marker + PnP/warp related terms
    5. 反向传播更新 Gaussian 参数
    6. 周期性执行 densification:
       - 高梯度小 Gaussian: clone
       - 高梯度大 Gaussian: split
       - 低 opacity 或过大 Gaussian: prune
    7. 保存 PLY / checkpoint / evaluation render

推理:
  - 渲染任务: 输入新相机位姿，输出 novel view
  - 几何任务: 输出 depth/normal/mesh
  - 定位任务: 2D feature ↔ 3D Gaussian feature matching，PnP + refinement
```

---

## 14. 参考文献

以下条目优先采用正式发表版本；尚未正式出版或以项目插件形式发布的工作保留 arXiv / 项目页信息。

1. Kerbl, B., Kopanas, G., Leimkuehler, T., & Drettakis, G. (2023). 3D Gaussian Splatting for Real-Time Radiance Field Rendering. *ACM Transactions on Graphics*, 42(4). [DOI:10.1145/3592433](https://doi.org/10.1145/3592433)
2. Zhang, B., Fang, C., Shrestha, R., Liang, Y., Long, X., & Tan, P. (2026). RaDe-GS: Rasterizing Depth in Gaussian Splatting. *ACM Transactions on Graphics*, 45(2), 1-14. [DOI:10.1145/3789201](https://doi.org/10.1145/3789201), [arXiv:2406.01467](https://arxiv.org/abs/2406.01467)
3. Ye, V., Li, R., Kerr, J., Turkulainen, M., Yi, B., Pan, Z., Seiskari, O., Ye, J., Hu, J., Tancik, M., & Kanazawa, A. (2025). gsplat: An Open-Source Library for Gaussian Splatting. *Journal of Machine Learning Research*, 26(34), 1-17. [JMLR](https://www.jmlr.org/papers/v26/24-1476.html)
4. Zhai, H., Zhang, X., Zhao, B., Li, H., He, Y., Cui, Z., Bao, H., & Zhang, G. (2025). SplatLoc: 3D Gaussian Splatting-based Visual Localization for Augmented Reality. *IEEE Transactions on Visualization and Computer Graphics*, 31(5), 3591-3601. [DOI:10.1109/TVCG.2025.3549563](https://doi.org/10.1109/TVCG.2025.3549563)
5. Sidorov, G., Mohrat, M., Gridusov, D., Rakhimov, R., & Kolyubin, S. (2025). GSplatLoc: Grounding Keypoint Descriptors into 3D Gaussian Splatting for Improved Visual Localization. In *2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 12601-12607. [DOI:10.1109/IROS60139.2025.11246406](https://doi.org/10.1109/IROS60139.2025.11246406)
6. Tancik, M., Weber, E., Ng, E., Li, R., Yi, B., Kerr, J., Wang, T., Kristoffersen, A., Austin, J., Salahi, K., Ahuja, A., McAllister, D., & Kanazawa, A. (2023). Nerfstudio: A Modular Framework for Neural Radiance Field Development. In *ACM SIGGRAPH 2023 Conference Proceedings*. [DOI:10.1145/3588432.3591516](https://doi.org/10.1145/3588432.3591516), [arXiv:2302.04264](https://arxiv.org/abs/2302.04264)
7. Xu, C., Kerr, J., & Kanazawa, A. (2024). Splatfacto-W: A Nerfstudio Implementation of Gaussian Splatting for Unconstrained Photo Collections. [arXiv:2407.12306](https://arxiv.org/abs/2407.12306)
8. Zhang, D., Wang, C., Wang, W., Li, P., Qin, M., & Wang, H. (2024). Gaussian in the Wild: 3D Gaussian Splatting for Unconstrained Image Collections. In *Computer Vision - ECCV 2024*, 341-359. [DOI:10.1007/978-3-031-73116-7_20](https://doi.org/10.1007/978-3-031-73116-7_20)
