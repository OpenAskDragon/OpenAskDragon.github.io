---
title: "StreetCrafter 算法逻辑与工程流程说明"
date: 2026-06-23
tags: [StreetCrafter, 3DGS, Video Diffusion, Autonomous Driving]
gitalk: false
---

# StreetCrafter 算法逻辑与工程流程说明

StreetCrafter 是一个面向自动驾驶街景新视角合成的研究型代码库。该项目将 LiDAR 条件视频扩散模型与动态 3D Gaussian 表示结合起来，用生成模型补充新轨迹监督，再将生成结果蒸馏到可直接渲染的 3D 表示中。该文档基于代码仓库中的实际实现整理，重点说明算法目标、数据流、训练流程、推理路径和主要工程边界。

## 项目链接

- 项目主页: [https://zju3dv.github.io/street_crafter](https://zju3dv.github.io/street_crafter)
- 论文: [StreetCrafter: Street View Synthesis with Controllable Video Diffusion Models](https://arxiv.org/abs/2412.13188)
- 源码仓库: [https://github.com/zju3dv/street_crafter](https://github.com/zju3dv/street_crafter)
- 基础 3D 表示: [Street Gaussians](https://zju3dv.github.io/street_gaussians/)
- 视频扩散基础代码: [Vista](https://opendrivelab.com/Vista/)

## 文档定位

该文档面向三类读者：

1. 需要快速理解 StreetCrafter 算法结构的研究者。
2. 需要复现 Waymo/PandaSet 数据处理、扩散采样和 3DGS 蒸馏流程的工程人员。
3. 需要修改渲染、损失、相机轨迹或数据集接口的二次开发者。

该文档不重复安装命令和权重下载流程；这些信息应以根目录 `README.md`、`data_processor/README.md` 和 `video_diffusion/docs/` 为准。

## 代码模块地图

| 模块 | 职责 | 关键产物 |
|------|------|----------|
| `data_processor/` | Waymo/PandaSet 原始数据转换、LiDAR 点云生成、LiDAR 条件渲染、meta JSON 生成 | `images/`、`ego_pose/`、`track/`、`lidar/`、`meta_info_*.json` |
| `create_scene.py` | 统一创建 dataset、Gaussian 模型、视频扩散模型和点云处理器 | `Scene`、`StreetGaussianModel`、`VideoDiffusionModel` |
| `street_gaussian/` | 动态 3D Gaussian 表示、相机读取、actor/background/sky 子模型、渲染与优化 | background/actor/sky Gaussians、renderer outputs、checkpoints |
| `video_diffusion/` | LiDAR 条件视频扩散训练与采样 | diffusion checkpoint、novel-view pseudo images |
| `train.py` | 动态 3DGS 蒸馏训练入口 | checkpoints、TensorBoard 日志、扩散伪监督缓存 |
| `render.py` | 输入轨迹渲染、新视角渲染、纯扩散推理 | trajectory videos、novel-view videos、diffusion videos |

## 端到端输入与输出

StreetCrafter 的实现可以被概括为一个跨 2D 视频生成和 3D 表示优化的流水线：

- 输入数据包括自动驾驶图像、相机标定、ego pose、LiDAR range image、动态目标 tracklet、可选 Colmap 稀疏点云和 sky mask。
- 中间条件包括彩色 LiDAR 渲染图、稀疏 LiDAR depth、动态目标 mask、新轨迹相机和扩散模型生成的伪监督图像。
- 输出结果包括可按输入轨迹或新轨迹渲染的动态 3D Gaussian 场景，以及用于调试或对比的视频扩散新视角结果。

该项目的关键设计不是单独追求视频生成质量，也不是只优化传统重建指标，而是让扩散模型承担“新视角外观补全”的角色，让 3D Gaussian 表示最终承担“显式场景渲染”的角色。

## 1. StreetCrafter 的总体算法目标

该代码库实现的是 StreetCrafter 的两阶段流程：

1. **可控视频扩散模型阶段**
   - 输入真实街景视频帧和 LiDAR 渲染条件图。
   - 训练或加载一个以 LiDAR 条件为控制信号的视频扩散模型。
   - 在新相机轨迹下，利用新轨迹 LiDAR 条件和参考真实图像生成新视角视频帧。

2. **动态 3D Gaussian 蒸馏阶段**
   - 用 LiDAR/Colmap 点云初始化动态街景 3D Gaussian。
   - 用真实训练视角监督 3DGS。
   - 在若干训练迭代点调用视频扩散模型，为新轨迹生成伪监督图像。
   - 将真实视角监督和扩散生成的新视角伪监督混合，优化一个可直接渲染的动态 3D 表示。

因此，StreetCrafter 不是单纯的视频生成系统，也不是单纯的 3DGS 重建系统。其核心逻辑是：

```text
原始自动驾驶数据
  -> LiDAR/tracklet/image/camera 标定预处理
  -> 条件视频扩散模型生成新视角伪图像
  -> 伪图像监督动态 3D Gaussian
  -> 得到可按任意支持轨迹渲染的动态街景表示
```

从工程角度看，该流程把困难拆成了两个互补问题：

- **几何约束**: LiDAR 条件图、稀疏深度、actor tracklet 和 camera pose 负责限制可见结构，降低扩散模型自由生成造成的几何漂移。
- **外观补全**: 视频扩散模型负责在新轨迹下补全遮挡、纹理和动态街景外观，为真实相机没有覆盖的位置提供伪监督。
- **显式渲染**: 动态 3D Gaussian 表示吸收真实视角和扩散伪视角监督，最终在推理阶段不再依赖扩散模型即可渲染新轨迹。

## 2. 数据预处理流程

### 2.1 原始数据转换

Waymo 数据预处理入口为 `data_processor/waymo_processor/waymo_converter.py`。

该脚本从 Waymo TFRecord 中提取：

- ego vehicle pose：保存到 `ego_pose/{frame:06d}.txt`。
- camera-synced pose：保存到 `ego_pose/{frame:06d}_{cam}.txt`。
- timestamp：保存到 `timestamps.json`。
- camera intrinsics/extrinsics：保存到 `intrinsics/{cam}.txt` 和 `extrinsics/{cam}.txt`。
- 图像：保存到 `images/{frame:06d}_{cam}.png`。
- 目标 tracklet：保存到 `track/track_info.pkl`、`track/track_camera_visible.pkl`、`track/trajectory.pkl` 和 `track/track_ids.json`。
- 动态目标 mask：保存到 `dynamic_mask/{frame:06d}_{cam}.png`。

tracklet 处理逻辑如下：

1. 遍历每帧 `laser_labels`。
2. 将 Waymo label 类型映射为 `vehicle`、`pedestrian`、`sign`、`cyclist` 或 `misc`。
3. 保存 LiDAR-synced 3D box 和可选 camera-synced 3D box。
4. 将 camera-synced box 投影到各相机，判断该目标在相机中是否可见。
5. 对每个 track 汇总轨迹、尺寸、速度、物体类别和每帧位姿。
6. 以世界坐标首尾距离和位置方差判断目标是否动态：
   - 若位置标准差任一轴大于 `0.5`，或首尾位移大于 `2` 米，则认为是动态目标。
   - 否则标记为 stationary。
7. 将 `pedestrian` 标记为 deformable，其他类别默认作为刚体对象。

动态 mask 的生成逻辑是：对每个相机可见的非静态目标，将其 3D box 投影成 2D mask，并将同帧同相机所有动态目标 mask 做逻辑或。

### 2.2 LiDAR 点云和稀疏深度生成

Waymo LiDAR 点云入口为 `data_processor/waymo_processor/waymo_get_lidar_pcd.py`。

核心步骤：

1. 从 Waymo v2 parquet 格式读取 LiDAR range image、LiDAR calibration、camera projection、LiDAR pose 和 vehicle pose。
2. 将 range image 转换为 3D 点云。
   - 有效 range 对应真实 LiDAR 点。
   - 缺失点使用 `DUMMY_DISTANCE_VALUE = 2e3` 参与占位处理，但最终有效点通过 range mask 过滤。
3. 从 LiDAR camera projection 中读取每个点投影到哪个相机，以及投影像素坐标。
4. 对每个相机生成稀疏 LiDAR 深度：
   - 将点变换到该相机坐标系。
   - 深度取相机 z 值。
   - 同一像素有多个点时用 `np.minimum.at` 做最小深度聚合，等价于稀疏 z-buffer。
   - 保存为 `lidar/depth/{frame:06d}_{cam}.npz`，其中包含 `mask` 和 `value`。
5. 对 LiDAR 点上色：
   - 根据 pointwise camera projection 找到可见相机和像素。
   - 从对应 RGB 图像采样颜色。
6. 按动态目标 3D box 将点云分为 actor 和 background：
   - 对每个动态目标，把 LiDAR 点变换到该 actor 的局部 box 坐标。
   - 判断点是否位于目标 box 内。
   - box 内点保存到 `lidar/actor/{track_id}/{frame:06d}.ply`。
   - 非 actor 点保存到 `lidar/background/{frame:06d}.ply`。
7. 每个 actor 还会汇总所有帧点云到 `lidar/actor/{track_id}/full.ply`。

这一阶段得到的 actor 点云在目标局部坐标中，background 点云仍按每帧车辆坐标存储，后续会再变换到世界坐标或当前目标位姿。

### 2.3 LiDAR 条件图渲染

离线渲染入口为 `data_processor/waymo_processor/waymo_render_lidar_pcd.py`，训练期间也会通过 `street_gaussian/pointcloud_processor/waymo_processor.py` 在线渲染条件图。

LiDAR 条件图用于视频扩散模型的控制信号。核心逻辑：

1. 读取 background 和 actor LiDAR PLY。
2. 对某一目标帧 `frame`，聚合 `[frame - delta_frames, frame + delta_frames]` 范围内的点云。
3. background 点：
   - 从对应帧车辆坐标变换到世界坐标。
4. actor 点：
   - 从 actor 局部坐标通过当前帧 tracklet box 位姿变换到世界坐标。
5. 新轨迹渲染时，根据 lane shift 方向对相机 ego pose 做横向平移。
6. 将点云变换到相机坐标，过滤视锥外和深度小于阈值的点。
7. 用 differentiable point rasterization 渲染得到：
   - `color_render/{frame:06d}_{cam}.png`: LiDAR 彩色条件图。
   - `color_render/{frame:06d}_{cam}_mask.png`: LiDAR 条件可见 mask。

离线脚本支持 `--shifts` 直接生成不同横向偏移轨迹的 LiDAR 条件。训练期间的 `PointCloudProcessor.render_condition()` 会在缺失或强制刷新时为当前 camera 自动生成对应条件图。

### 2.4 视频扩散训练元数据

`data_processor/waymo_processor/waymo_prepare_meta.py` 会生成类似如下结构的 JSON：

```json
{
  "frames": [
    "training_set_processed/000/images/000000_0.png"
  ],
  "guidances": [
    "training_set_processed/000/lidar/color_render/000000_0.png"
  ],
  "guidances_mask": [
    "training_set_processed/000/lidar/color_render/000000_0_mask.png"
  ]
}
```

`video_diffusion/vwm/data/subsets/waymo.py` 读取该 JSON，将真实图像序列 `img_seq` 和 LiDAR 条件序列 `guide_seq` 统一裁剪、缩放到目标分辨率，并归一化到 `[-1, 1]`。

## 3. 运行时 Scene 构建

统一入口为 `create_scene.py`。

```text
Dataset()
  -> StreetGaussianModel(dataset.metadata)
  -> 可选 VideoDiffusionModel
  -> 可选 PointCloudProcessor
  -> Scene(...)
```

### 3.1 Dataset 构建

当前 `street_gaussian/datasets/dataset.py` 的场景读取回调只注册了 `Waymo`：

```python
sceneLoadTypeCallbacks = {
    "Waymo": readWaymoInfo,
}
```

Waymo 场景读取逻辑在 `street_gaussian/datasets/waymo_readers.py`：

1. 调用 `generate_dataparser_outputs()` 读取标定、相机位姿、图片路径、tracklet、timestamp 和 Colmap 输出。
2. 根据 `split_train` 和 `split_test` 划分 train/test frame。
3. 为每张图构造 `CameraInfo`：
   - 外参：`c2w = ego_pose @ ext`，再取逆得到世界到相机的 `R, T`。
   - 内参：`K`。
   - 图像：PIL image。
   - metadata：frame、cam、frame_idx、ego_pose、extrinsic、timestamp、guidance 路径等。
   - guidance：动态 mask、sky mask、LiDAR depth 等训练辅助信号。
4. 调用 `waymo_novel_view_cameras()` 生成新视角 camera。
5. 计算 NeRF++ 风格 scene center/radius，写入 metadata。

`cameraList_from_camInfos()` 会将 `CameraInfo` 转成运行时 `Camera`：

- 图像和 guidance 按分辨率缩放。
- 构造 `world_view_transform`、`projection_matrix`、`full_proj_transform` 和 `camera_center`。
- 保存相机内外参、图像、metadata 和 guidance。

### 3.2 新视角相机生成

`street_gaussian/utils/novel_view_utils.py` 的 Waymo 新视角生成只使用前视相机 `cam == 0`。

每个 shift 模式下：

1. 复制原始前视相机。
2. 设置 `metadata['is_novel_view'] = True`。
3. 设置 `metadata['novel_view_id'] = shift`。
4. 根据 `get_lane_shift_direction_waymo()` 得到车道横向方向。
5. 用 `LANE_SHIFT_SIGN` 决定具体左右方向。
6. 对 ego pose 的平移项加上 `shift_direction * shift * sign`。
7. 重新计算新相机的 `R, T`。
8. 设置新轨迹 LiDAR 条件图路径，例如 `lidar/color_render_shift_2.00/{frame}_{cam}.png`。
9. 若目标离新相机过近，则设置 `skip_camera=True`，训练时跳过该新视角。

训练模式下会过滤 `shift == 0` 的新视角，因为原轨迹已有真实监督。

### 3.3 Scene 初始化

`street_gaussian/models/scene.py` 根据运行模式执行不同逻辑：

- `mode == train`
  - 初始化点云处理器。
  - 调用 `pointcloud_processor.initailize_ply(cfg.model_path, obj_meta)` 生成 `input_ply`。
  - 调用 `gaussians.create_from_pcd()` 从 PLY 初始化所有 Gaussian 子模型。
  - 将训练相机和新视角相机图像/guidance 移到 CUDA。
  - 如果启用扩散，还会提前渲染 LiDAR 条件。

- `mode == diffusion`
  - 初始化点云处理器和视频扩散模型。
  - 渲染 train/test/novel cameras 的 LiDAR 条件。

- 其他渲染模式
  - 从 `cfg.trained_model_dir` 加载指定或最新 checkpoint。

## 4. 动态 Street Gaussian 表示

核心类为 `street_gaussian/models/street_gaussian_model.py` 的 `StreetGaussianModel`。

### 4.1 子模型组成

模型根据配置拆分为多个可独立优化的部分：

1. **background Gaussian**
   - 类：`GaussianModelBkgd`。
   - 从 `input_ply/points3D_bkgd.ply` 初始化。
   - 若 `data.use_colmap=True`，初始化点云会合并 LiDAR background 和 Colmap sparse points。

2. **actor Gaussian**
   - 类：`GaussianModelActor`。
   - 每个动态对象一个子模型，命名为 `obj_{object_id:03d}`。
   - 点云在 actor 局部坐标系中。
   - 从 `input_ply/points3D_obj_{object_id:03d}.ply` 初始化；如果点数不足或文件不存在，退化为 box 内规则网格随机颜色初始化。
   - 刚体对象可使用左右翻转对称先验，pedestrian 等 deformable 对象不使用翻转。

3. **sky**
   - 二选一：
     - `include_sky=True`: 使用 `GaussianModelSky`，从 `points3D_sky.ply` 初始化远处天空 Gaussian。
     - `include_cube_map=True`: 使用可学习 `SkyCubeMap`，按视线方向采样 cubemap 颜色。
   - 示例配置默认 `include_sky=False`、`include_cube_map=True`。

4. **actor pose**
   - 类：`ActorPose`。
   - 保存每个相机、每帧、每个 actor 的 tracklet translation 和 quaternion。
   - 若 `model.nsg.opt_track=True`，额外学习 `opt_trans` 和 `opt_rots` 修正 tracklet。

5. **color correction**
   - 类：`ColorCorrection`。
   - 可选；默认关闭。
   - 支持 per-image 或 per-sensor 3x4 仿射颜色变换。

6. **pose correction**
   - 类：`PoseCorrection`。
   - 可选；默认关闭。

### 4.2 单个 Gaussian 的参数化

基础类 `GaussianModel` 中，每个 3D Gaussian 包含：

- `_xyz`: 三维中心。
- `_features_dc`: 球谐颜色 DC 分量。
- `_features_rest`: 其余球谐颜色分量。
- `_scaling`: log-space scale，实际 scale 为 `exp(_scaling)`。
- `_rotation`: quaternion，渲染前归一化。
- `_opacity`: inverse-sigmoid 空间 opacity，实际 opacity 为 `sigmoid(_opacity)`。
- `_semantic`: 可选语义参数。

初始化时：

- 颜色先转为 spherical harmonics 的 DC 分量。
- scale 根据最近邻距离 `distCUDA2` 初始化为 `log(sqrt(dist2))`。
- rotation 初始化为单位 quaternion。
- opacity 初始化为 `inverse_sigmoid(0.1)`。

### 4.3 Actor 的时变颜色

`GaussianModelActor.get_features_fourier(frame)` 对 actor 的 DC 颜色使用 Fourier/IDFT 形式的时间调制：

1. 将当前帧归一化到 actor 生命周期 `[start_frame, end_frame]`。
2. 计算 `time = fourier_scale * normalized_frame`。
3. 用 `IDFT(time, fourier_dim)` 得到基函数。
4. 将 `_features_dc` 沿 Fourier 维度加权求和，得到当前帧 DC 颜色。
5. 与静态的 `_features_rest` 拼接成当前帧 SH 特征。

如果 `fourier_dim=1`，该机制退化为近似静态颜色。

### 4.4 当前相机下的场景图解析

渲染前必须调用 `StreetGaussianModel.parse_camera(camera)`。

该函数完成：

1. 设置当前 camera、frame、frame_idx、validation flag。
2. 根据当前可见集合决定启用 background、哪些 actor、sky。
3. 对每个 actor：
   - 判断当前 frame 是否位于 actor `[start_frame, end_frame]`。
   - 如果可见，则加入 `graph_obj_list`。
   - 通过 `ActorPose` 获取当前帧 actor 的 world rotation 和 translation。
4. 记录每个子模型在拼接后 Gaussian 数组中的 index range，用于 densification 统计回写。
5. 训练时可对非 deformable actor 随机应用局部 y 轴翻转，作为刚体对称增强。

`get_xyz`、`get_rotation`、`get_features`、`get_opacity` 等属性会按当前场景图拼接所有可见子模型。

Actor 坐标变换为：

```text
xyz_world = R_actor_world * xyz_actor_local + t_actor_world
rot_world = q_actor_world * q_actor_local
```

因此 actor Gaussian 的几何在局部坐标中优化，而在每帧通过 tracklet/learned correction 放置到世界坐标。

## 5. Gaussian 渲染算法

核心类为 `street_gaussian/models/street_gaussian_renderer.py` 的 `StreetGaussianRenderer`。

### 5.1 渲染入口

主要接口：

- `render()`: 渲染完整组合结果。
- `render_background()`: 只渲染 background。
- `render_object()`: 只渲染 actors。
- `render_sky()`: 只渲染 sky Gaussian。
- `render_all()`: 同时输出组合、background、object 分量。
- `render_novel_view()`: 新视角渲染，逻辑与 `render()` 接近，但推理式 clamp。

完整渲染默认排除 `sky`，先渲染 foreground/background Gaussian，再将 sky 按透明度补到未覆盖区域：

```text
rgb_final = rgb_gaussian + rgb_sky * (1 - acc_gaussian)
```

如果启用 cubemap，`rgb_sky` 来自 `SkyCubeMap(camera, acc)`；如果启用 sky Gaussian，则单独调用 `render_sky()`。

### 5.2 gsplat 渲染路径

当 `cfg.render.use_gsplat=True` 时，使用 `render_kernel_gsplat()`：

1. 获取当前拼接后的 Gaussian 参数：
   - `xyz3`
   - `rgb3`/SH features
   - `scale`
   - `quats`
   - `opacity`
2. 构造当前相机 `w2c` 和 `K`。
3. 调用 `fully_fused_projection()`：
   - 将 3D Gaussian 投影到屏幕空间。
   - 得到 `radii`、`means2d`、`depths`、`conics` 和抗锯齿 compensation。
4. 根据 tile size 调用 `isect_tiles()` 和 `isect_offset_encode()`，建立 Gaussian 与屏幕 tile 的相交关系。
5. 用 spherical harmonics 根据视线方向求颜色：
   - `dirs = xyz - camera_center`
   - `colors = spherical_harmonics(max_sh_degree, dirs, shs, masks)`
6. 将 depth 拼入 colors 作为额外通道。
7. 调用 `rasterize_to_pixels()` 做 alpha compositing。
8. 输出：
   - `rgb`: `[3, H, W]`
   - `acc`: `[1, H, W]`
   - `depth`: `[1, H, W]`
   - `viewspace_points`: 屏幕空间均值，用于 densification 梯度统计。
   - `visibility_filter`: `radii > 0`
   - `radii`: 屏幕半径归一化值。

### 5.3 diff-gaussian-rasterization 路径

当 `cfg.render.use_gsplat=False` 时，使用 `render_kernel_diff_gauss()`：

1. 通过 `make_rasterizer()` 构造 `GaussianRasterizer`。
2. 准备 `means3D`、`means2D`、opacity、scale/rotation 或 precomputed covariance。
3. 颜色可以由 Python 预计算 SH，也可以交给 rasterizer。
4. rasterizer 输出 RGB、radii、depth、acc。

该路径同样返回用于训练和 densification 的可见性与屏幕空间点。

## 6. 条件视频扩散模型

扩散模型主要位于 `video_diffusion/`。

### 6.1 训练数据

`video_diffusion/vwm/data/subsets/waymo.py` 的 `WaymoDataset` 返回：

- `img_seq`: 真实视频帧序列，shape 约为 `[T, 3, H, W]`，范围 `[-1, 1]`。
- `guide_seq`: LiDAR 彩色条件序列，范围 `[-1, 1]`。
- `motion_bucket_id`: 默认 `127`。
- `fps_id`: 默认 `9`，对应 10 FPS。
- `cond_frames_without_noise`: 第 0 帧真实图。
- `cond_frames`: 第 0 帧加条件增强噪声后的图；当前 `cond_aug=0`。

### 6.2 DiffusionEngine 训练逻辑

`video_diffusion/vwm/models/diffusion_condition.py` 中的 `DiffusionEngine` 包含：

- `first_stage_model`: VAE/autoencoder，负责图像和 latent 互转，参数冻结。
- `model`: diffusion UNet，被 `OPENAIUNETWRAPPER` 包装。
- `denoiser`: EDM denoiser。
- `conditioner`: 处理条件 embedding。
- `loss_fn`: 标准扩散损失。
- 可选 EMA。

训练 step：

1. `get_input(batch)` 将 `[B, T, C, H, W]` 展平为 `[(B*T), C, H, W]`。
2. `encode_first_stage()` 将真实图像编码为 latent `x`。
3. 若 batch 中有 `guide`，调用 `get_guidance()`：
   - 将 LiDAR 条件图编码为 latent。
   - 生成 guidance scale。
   - 以 `ucg_rate=0.15` 随机丢弃部分 guidance，类似 classifier-free guidance 的训练增强。
4. 调用 `StandardDiffusionLoss`。

### 6.3 标准扩散损失

`video_diffusion/vwm/modules/diffusionmodules/loss.py` 的 `StandardDiffusionLoss` 逻辑：

1. 为每个 latent sample 采样噪声级别 `sigma`。
2. 采样标准高斯噪声 `noise`。
3. 构造带噪输入：

```text
x_noised = x + sigma * noise
```

4. 如果启用 `replace_cond_frames`，条件帧不加噪或在 cond mask 中被替换。
5. 调用 denoiser 预测 clean latent。
6. 用 `loss_weighting(sigma)` 对不同噪声级别加权。
7. 支持 L2 或 L1 损失。
8. 可选 additional loss：
   - temporal difference auxiliary loss。
   - high-frequency Fourier loss。

默认主损失是预测 latent 和真实 latent 之间的加权逐样本重建损失。

### 6.4 推理封装 VideoDiffusionModel

`video_diffusion/sample_condition.py` 的 `VideoDiffusionModel` 负责加载配置和 checkpoint，并执行采样。

初始化时：

1. 加载 OmegaConf config。
2. 实例化 `DiffusionEngine`。
3. 加载 `.ckpt` 或 `.safetensors` 权重。
4. 初始化 `EulerEDMSamplerSDS`。
5. 设置采样分辨率、随机种子、`cond_aug` 等参数。

`forward(batch, scale, cond_indices)` 的流程：

1. 固定随机种子。
2. 补齐 `guide_mask_seq` 和 `img_mask_seq`。
3. 构造 `value_dict`：
   - 第 0 帧作为 `cond_frames_without_noise` 和 `cond_frames`。
   - `guide_frames` 为 LiDAR 条件序列。
   - 若 `training_free_guidance=True`，还加入当前 Gaussian render 的图像 latent 和 mask。
4. `get_condition()` 生成 conditional/unconditional conditioning。
5. 将 `img_frames` 编码成 latent `z`。
6. 根据 `cond_indices` 构造 `cond_mask`，保证条件帧在采样过程中被替换为给定 latent。
7. 用 `EulerEDMSamplerSDS` 从噪声或带噪 render latent 开始迭代去噪。
8. 将输出 latent 解码为 RGB，映射到 `[0, 1]`。

### 6.5 训练外引导采样

`EulerEDMSamplerSDS.__call__()` 对 `sample_guidance` 有特殊逻辑：

- 如果 `cond` 中存在 `sample_guidance`，说明传入了当前 3DGS 的 render latent。
- 采样步数会从后半段开始：

```text
num_inference_steps = int(num_steps * scale)
start_step = num_steps - num_inference_steps
x = render_latents + noise * sigmas[start_step]
```

也就是说，当前 Gaussian 渲染不是作为最终硬约束混合，而是作为带噪初值，让扩散模型从该渲染附近修正到更真实的新视角结果。文件中有更细粒度 mask blending 的注释代码，但该实现没有启用该 blending 分支。

## 7. 扩散与 3DGS 的连接

连接类为 `street_gaussian/utils/diffusion_utils.py` 的 `DiffusionRunner` 和 `WaymoDiffusionRunner`。

### 7.1 LiDAR guidance 准备

`get_guidance(cameras)` 会：

1. 调用 `pointcloud_processor.render_conditions(cameras, obj_meta)`。
2. 确保每个 camera 的 `guidance_rgb_path` 和 `guidance_mask_path` 已存在。
3. 返回条件图路径列表。

### 7.2 当前 Gaussian render 准备

当 `use_render=True`：

1. 对新视角 camera 调用 `renderer.render_novel_view(camera, gaussians)`。
2. 收集 `render_seq` 和 `render_mask_seq`。
3. 在 diffusion 采样时，这些 render 会被 resize/crop 到扩散分辨率，范围转为 `[-1, 1]`。
4. batch 中设置：

```python
batch["training_free_guidance"] = True
batch["masked_guidance"] = masked_guidance
```

### 7.3 新视角序列扩散生成

`WaymoDiffusionRunner.run_sequence()` 的逻辑：

1. 默认只处理前视相机 `cam == 0`。
2. 对同一个 `novel_view_id` 的 camera 按 frame 排序。
3. 设：
   - `model_num_frames = scene.diffusion.num_frames`
   - `sample_frames = model_num_frames - 1`
   - `step = sample_frames - cfg.diffusion.window_size`
4. 用滑动窗口覆盖整个新视角序列。
5. 每个窗口中：
   - 找到与窗口起始 frame 最近的真实训练帧。
   - 将该真实训练帧作为第 0 条件帧。
   - 将该真实帧的 LiDAR 条件放在 guide 序列第 0 个。
   - 将窗口内新视角 LiDAR 条件接在后面。
   - 如果 `use_render=True`，将真实条件图像和窗口内当前 Gaussian render 拼成 `img_seq`。
   - 调用 `VideoDiffusionModel.forward(..., cond_indices=[0])`。
   - 丢弃输出第 0 条件帧，只保留后续新视角输出。
6. 将生成结果写回每个 camera：

```python
camera.meta["diffusion_original_image"] = diffusion_result[i]
```

该字段随后在 3DGS 训练中作为新视角伪 GT。

## 8. 3D Gaussian 蒸馏训练流程

入口为 `train.py` 的 `training()`。

### 8.1 初始化

1. 读取配置：
   - `cfg.train`
   - `cfg.optim`
   - `cfg.data`
   - `cfg.diffusion`
2. 创建输出目录、TensorBoard writer。
3. 调用 `create_scene()`：
   - 构造 dataset。
   - 构造 Gaussian model。
   - 构造 video diffusion model。
   - 构造 pointcloud processor。
   - 初始化 PLY 和 Gaussian。
4. 调用 `gaussians.training_setup()`：
   - 每个 Gaussian 子模型建立独立 Adam optimizer。
   - actor pose、sky cubemap、color correction、pose correction 也各自建立 optimizer。
   - active SH degree 初始为 0。
5. 如果 `cfg.trained_model_dir` 中已有 checkpoint，则加载最新或指定 iteration。

### 8.2 每轮迭代主循环

对 `iteration in [start_iter, cfg.train.iterations]`：

1. 更新所有子模块 learning rate。
2. 每 1000 轮提升一次 SH degree，直到最大值。
3. 在指定 diffusion sampling iteration 运行扩散生成新视角伪监督。
4. 从真实训练相机和已生成的新视角相机中随机采样一个 viewpoint。
5. 调用 renderer 渲染当前 viewpoint。
6. 根据 viewpoint 是真实视角还是新视角，计算不同损失。
7. `loss.backward()`。
8. 记录可视化和训练指标。
9. 在 densification 阶段统计屏幕空间梯度和半径，执行 clone/split/prune。
10. 周期性 reset opacity。
11. 在测试 iteration 评估 train/test view。
12. optimizer step。
13. 在 checkpoint iteration 保存模型。

### 8.3 扩散采样触发策略

是否使用扩散由如下条件决定：

```python
use_diffusion = cfg.diffusion.use_diffusion and len(cfg.diffusion.sample_iterations) > 0
```

示例 Waymo 配置中：

```yaml
diffusion:
  use_diffusion: True
  sample_iterations: [7000, 12000, 17000, 22000]
  sample_scales: [0.7, 0.3]
```

在这些 iteration 上：

1. 取全部 novel view cameras。
2. 根据当前 iteration 线性计算 `scale`：

```text
scale = (min_scale - max_scale) * (iteration - min_iteration)
        / (max_iteration - min_iteration)
        + max_scale
```

若 `sample_scales=[0.7, 0.3]`，则早期 scale 约为 `0.7`，后期逐步降到 `0.3`。在该采样器中，scale 越小，采样越从更靠后的扩散步开始，也就是越贴近当前 Gaussian render 初值。

3. 调用：

```python
diffusion_runner.run(
    novel_viewpoint_stack,
    train_viewpoint_stack,
    use_render=True,
    scale=scale,
    masked_guidance=iteration >= cfg.diffusion.masked_guidance_iter,
)
```

4. 用新生成结果替换旧的新视角伪 GT。
5. 将未被 `skip_camera` 标记的新视角相机加入 `viewpoint_stack`。

### 8.4 训练相机采样

训练时维护：

- `train_viewpoint_stack`: 真实训练相机。
- `viewpoint_stack`: 真实训练相机 + 扩散生成过伪 GT 的新视角相机。
- `training_camera_number`: 真实训练相机数量。

每轮以 `cfg.train.novel_view_prob` 决定是否采样新视角。默认示例配置继承默认值 `0.4`。

若决定采样新视角且已有新视角伪 GT，则从 `[training_camera_number, len(viewpoint_stack))` 范围采样；否则从真实训练相机采样。

### 8.5 真实视角损失

真实视角使用真实图像 `viewpoint_cam.original_image` 作为 GT。

基础重建损失：

```text
L_real =
  (1 - lambda_dssim) * lambda_l1 * L1(render, gt, mask)
  + lambda_dssim * (1 - SSIM(render, gt, mask))
  + lambda_lpips * LPIPS(render * mask, gt * mask)
```

其中 mask 优先使用 `viewpoint_cam.guidance['mask']`，不存在时为全 1。

可选正则项：

1. **sky loss**
   - 条件：`lambda_sky > 0`，且启用 sky/cubemap，且存在 `sky_mask`。
   - 对 sky 区域鼓励 Gaussian acc 低，对非 sky 区域使用 entropy 风格约束。
   - 可按相机使用 `lambda_sky_scale` 缩放。

2. **object accumulation regularization**
   - 条件：`lambda_reg > 0`，启用 object，存在 `obj_bound`，且 iteration 满足代码条件。
   - 单独渲染 object acc。
   - 在 object bound 外鼓励 object acc 低，在 bound 内用 entropy 风格约束。

3. **LiDAR depth loss**
   - 条件：`lambda_depth_lidar > 0` 且存在 `lidar_depth`。
   - 用有效 LiDAR depth 像素约束渲染 depth。
   - 仅保留误差较小的 95% 点求均值，以降低离群点影响。

4. **scale flatten loss**
   - 条件：`lambda_scale_flatten > 0`。
   - 鼓励 Gaussian 更扁平，减少异常厚度。

5. **color correction regularization**
   - 条件：`lambda_color_correction > 0` 且启用 color correction。
   - 约束颜色仿射变换接近 identity。

### 8.6 新视角伪监督损失

新视角使用扩散生成结果作为 GT：

```python
gt_image = viewpoint_cam.meta["diffusion_original_image"]
```

流程：

1. 将 Gaussian render、mask resize/crop 到扩散模型分辨率。
2. 取图像下方 60% 区域计算损失：
   - 代码中 `upper = int(mask.shape[-2] * 0.4)`。
   - 使用 `image[:, upper:, :]` 和 `gt_image[:, upper:, :]`。
   - 上方区域不参与新视角伪监督，主要避免天空/远景不稳定区域主导训练。
3. 损失：

```text
L_novel =
  lambda_novel * (
    (1 - lambda_novel_dssim) * lambda_novel_l1 * L1
    + lambda_novel_dssim * (1 - SSIM)
    + lambda_novel_lpips * LPIPS
  )
```

示例默认权重中 `lambda_novel=0.1`，因此新视角伪监督相对真实视角更弱。

### 8.7 Densification 与 pruning

训练前期执行 densification，默认：

- `densify_from_iter = 500`
- `densify_until_iter = 15000`
- `densification_interval = 100`
- `opacity_reset_interval = 3000`

每轮渲染后：

1. 对可见 Gaussian 更新最大屏幕半径 `max_radii2D`。
2. 累计屏幕空间均值梯度：
   - 普通梯度使用 `viewspace_point_tensor.grad`。
   - 如果 gsplat 提供 `absgrad`，则同时利用 abs gradient。
3. 每到 densification interval：
   - 对高梯度、小 scale 的点执行 clone。
   - 对高梯度、大 scale 的点执行 split。
   - 删除 opacity 低于 `min_opacity` 的点。
   - 可选删除世界空间或屏幕空间过大的点。

Background、actor、sky 的 pruning 规则略有不同：

- background 使用 scene radius 和 LiDAR sphere 范围判断过大点。
- actor 还会随机采样 Gaussian 分布点，检查是否超出 actor box。
- sky 会把点限制在远处球面附近，并限制 scale 不超过 sphere radius。

### 8.8 优化器结构

每个 Gaussian 子模型有独立 Adam optimizer，参数组包括：

- xyz
- f_dc
- f_rest
- opacity
- scaling
- rotation
- semantic

额外模块也有独立 optimizer：

- `ActorPose`: tracklet translation/rotation correction。
- `SkyCubeMap`: cubemap texture。
- `ColorCorrection`: affine color transform。
- `PoseCorrection`: camera pose correction。

训练主循环中的 `gaussians.update_optimizer()` 会逐个子模块 step 并清梯度。

## 9. 推理和渲染流程

入口为 `render.py`，由 `cfg.mode` 控制。

### 9.1 `mode trajectory`

`render_trajectory()`：

1. 创建 scene 并加载训练好的 checkpoint。
2. 合并 train cameras 和 test cameras。
3. 按 camera id 排序。
4. 对每个 camera 调用 `renderer.render_all()`。
5. 保存组合 RGB、background RGB、object RGB、object acc、depth、diff 等视频或图像。

该模式用于复现输入轨迹上的重建质量。

### 9.2 `mode novel_view`

`render_novel_view()`：

1. 创建 scene 并加载 checkpoint。
2. 读取配置生成的新视角 cameras。
3. 按 `novel_view_id` 分组。
4. 每组按 frame 排序。
5. 对每个新视角 camera 调用 `renderer.render_novel_view()`。
6. 保存新视角 RGB、acc、depth 视频。

该模式不再调用扩散模型，而是直接用已经蒸馏好的 3DGS 渲染。

### 9.3 `mode diffusion`

`run_diffusion()`：

1. 创建 scene。
2. 获取 train cameras 和 novel view cameras。
3. 构造 `DiffusionRunner`。
4. 对每个 `novel_view_id`：
   - 按 frame 排序。
   - 调用扩散模型生成新视角视频。
   - 保存为 `model_path/diffusion/diffusion_novel_{novel_view_id}.mp4`。

该模式用于只运行视频扩散模型，不进行 3DGS 蒸馏优化。

## 10. 示例 Waymo 配置含义

示例父配置为 `configs/waymo_val_121.yaml`。

关键设置：

- 数据：
  - `data.type: Waymo`
  - `data.cameras: [0]`，只使用前视相机。
  - `data.selected_frames: [98, 198]`
  - `data.extent: 20.`
  - `data.use_colmap: True`

- Gaussian：
  - `sh_degree: 1`
  - `fourier_dim: 1`
  - `flip_prob: 0.2`
  - `include_bkgd: True`
  - `include_obj: True`
  - `include_sky: False`
  - `include_cube_map: True`
  - `opt_track: True`

- 训练：
  - `iterations: 30000`
  - `test_iterations: [7000, 30000]`
  - `checkpoint_iterations: [7000, 30000]`

- 损失：
  - `lambda_dssim: 0.2`
  - `lambda_reg: 0.1`
  - `lambda_sky: 0.05`
  - `lambda_depth_lidar: 0.01`
  - `lambda_lpips: 0.5`
  - `lambda_novel: 0.1`

- 扩散：
  - `use_diffusion: True`
  - `height: 576`
  - `width: 1024`
  - `sample_iterations: [7000, 12000, 17000, 22000]`
  - `sample_scales: [0.7, 0.3]`

其他 scene 配置如 `waymo_val_049.yaml`、`waymo_val_096.yaml` 等继承该父配置，只改 `source_path`、`exp_name`、`selected_frames`，部分场景关闭 `include_cube_map`。

## 11. 端到端流程图

```text
Waymo 原始数据
  |
  |-- waymo_converter.py
  |     -> images / ego_pose / intrinsics / extrinsics
  |     -> track_info / trajectory / dynamic_mask
  |
  |-- waymo_get_lidar_pcd.py
  |     -> lidar/background/*.ply
  |     -> lidar/actor/{track_id}/*.ply
  |     -> lidar/depth/*.npz
  |
  |-- waymo_render_lidar_pcd.py 或 PointCloudProcessor.render_condition()
  |     -> lidar/color_render/*.png
  |     -> lidar/color_render/*_mask.png
  |
  |-- waymo_prepare_meta.py
  |     -> meta_info_train.json / meta_info_val.json
  |
  +--> 视频扩散模型训练或加载 checkpoint
  |
  +--> train.py
        |
        |-- Dataset/readWaymoInfo
        |     -> train/test/novel cameras
        |
        |-- PointCloudProcessor.initailize_ply
        |     -> model_path/input_ply/points3D_bkgd.ply
        |     -> model_path/input_ply/points3D_obj_*.ply
        |     -> 可选 points3D_sky.ply
        |
        |-- StreetGaussianModel.create_from_pcd
        |     -> background Gaussian
        |     -> actor Gaussians
        |     -> sky/cubemap
        |
        |-- 迭代优化
              |
              |-- 真实训练视角:
              |     render -> RGB/SSIM/LPIPS/depth/regularization loss
              |
              |-- 指定迭代点:
              |     novel cameras + LiDAR conditions + current render
              |       -> video diffusion
              |       -> diffusion_original_image
              |
              |-- 新视角:
              |     render -> 与 diffusion_original_image 做伪监督损失
              |
              |-- densify / prune / opacity reset / checkpoint
```

## 12. 公式与代码对应关系

本节把核心公式、实现位置和变量名对应起来，便于从论文式描述追踪到代码实现。公式使用的符号与代码变量不完全同名；下表优先列出代码中的实际张量名。

### 12.1 Gaussian 参数化

| 公式或逻辑 | 代码位置 | 代码变量 |
|------------|----------|----------|
| Gaussian 中心直接优化：$\mu_i = \_xyz_i$ | `street_gaussian/models/gaussian_model.py::get_xyz` | `_xyz`, `get_xyz` |
| log-space scale 转实际尺度：$s_i = \exp(\_scaling_i)$ | `street_gaussian/models/gaussian_model.py::setup_functions`, `get_scaling` | `_scaling`, `scaling_activation`, `get_scaling` |
| quaternion 归一化：$q_i = \operatorname{normalize}(\_rotation_i)$ | `street_gaussian/models/gaussian_model.py::get_rotation` | `_rotation`, `rotation_activation`, `get_rotation` |
| opacity 激活：$\alpha_i = \sigma(\_opacity_i)$ | `street_gaussian/models/gaussian_model.py::get_opacity` | `_opacity`, `opacity_activation`, `get_opacity` |
| 协方差由尺度和旋转构成：$\Sigma_i = R(q_i)\,\operatorname{diag}(s_i)^2 R(q_i)^T$ | `street_gaussian/models/gaussian_model.py::get_covariance` | `get_scaling`, `_rotation`, `build_covariance_from_scaling_rotation` |
| 初始化尺度：$\_scaling_i = \log\sqrt{d_i^2}$，其中 $d_i^2$ 来自最近邻距离 | `GaussianModel.create_from_pcd`, `GaussianModelActor.create_from_pcd` | `distCUDA2`, `dist2`, `scales` |
| 初始化 opacity：$\_opacity_i = \operatorname{logit}(0.1)$ | `GaussianModel.create_from_pcd`, `GaussianModelActor.create_from_pcd` | `inverse_sigmoid(0.1)`, `opacities` |

基础 Gaussian 的颜色以球谐系数表示。静态 background 使用 `_features_dc + _features_rest`；actor 的 DC 颜色额外带时间 Fourier 调制。

### 12.2 Actor 时序颜色与世界坐标变换

actor Gaussian 在局部坐标中优化，渲染时根据 tracklet/learned pose 放置到世界坐标：

$$
x_{world} = R_{actor}(t) x_{local} + t_{actor}(t)
$$

$$
q_{world} = q_{actor}(t) \otimes q_{local}
$$

对应实现：

- `street_gaussian/models/street_gaussian_model.py::parse_camera()` 从 `ActorPose` 读取 `obj_rot` 和 `obj_trans`。
- `StreetGaussianModel.get_xyz` 用 `torch.einsum('bij, bj -> bi', obj_rots, xyzs_local) + self.obj_trans` 计算 actor 世界坐标。
- `StreetGaussianModel.get_rotation` 用 `quaternion_raw_multiply(self.obj_rots, rotations_local)` 合成世界旋转。

actor 的时间变化颜色使用 IDFT 基函数调制 DC 球谐项：

$$
\tau = \text{fourier\_scale}\cdot\frac{frame-start\_frame}{end\_frame-start\_frame}
$$

$$
f_{dc}(frame)=\sum_k a_k\,\operatorname{IDFT}_k(\tau)
$$

对应实现为 `street_gaussian/models/gaussian_model_actor.py::get_features_fourier()`：

- `normalized_frame = (frame - self.start_frame) / (self.end_frame - self.start_frame)`
- `idft_base = IDFT(time, self.fourier_dim)[0]`
- `features_dc = torch.sum(features_dc * idft_base[..., None], dim=1, keepdim=True)`

当 `fourier_dim=1` 时，只有一个 DC 系数参与求和，actor 颜色近似退化为静态颜色。

### 12.3 gsplat 渲染路径

渲染器先从 `StreetGaussianModel` 取当前相机场景图下的拼接参数：

| 数学对象 | 代码变量 | 来源 |
|----------|----------|------|
| Gaussian 中心 $\mu$ | `xyz3` | `pc.get_xyz` |
| SH 颜色系数 | `rgb3` | `pc.get_features` |
| 尺度 $s$ | `scale` | `pc.get_scaling` |
| 旋转 $q$ | `quats` | `pc.get_rotation` |
| opacity $\alpha$ | `occ1`, `opacities` | `pc.get_opacity` |
| 相机外参/内参 | `w2c`, `K` | `camera.world_view_transform`, `camera.K` |

`street_gaussian/models/street_gaussian_renderer.py::render_kernel_gsplat()` 的主要步骤是：

1. `fully_fused_projection()` 将 3D Gaussian 投影到屏幕，得到 `radii`、`means2d`、`depths`、`conics`。
2. `isect_tiles()` 和 `isect_offset_encode()` 构建 Gaussian 与屏幕 tile 的相交索引。
3. `spherical_harmonics()` 根据相机视线方向 `dirs = xyz3 - camera_center` 计算 RGB。
4. `rasterize_to_pixels()` 执行 alpha compositing，输出 `render_colors` 和 `render_alphas`。

若启用 depth 通道，代码将 `depths` 拼到颜色通道后渲染，并在输出时做归一化：

$$
D(u,v)=\frac{\sum_i T_i\alpha_i d_i}{\sum_i T_i\alpha_i + \epsilon}
$$

对应代码：

- `colors = torch.cat((colors, depths[..., None]), dim=-1)`
- `rendered_depth = render_colors[..., -1:] / render_alphas.clamp(min=1e-10)`
- `rendered_acc = render_alphas`

天空合成使用标准前景 alpha 补背景形式：

$$
I_{final}=I_{gaussian}+I_{sky}(1-A_{gaussian})
$$

对应代码为 `StreetGaussianRenderer.render()` 中的：

- `result['rgb'] = result['rgb'] + result_sky['rgb'] * (1 - result['acc'])`
- 或 cubemap 分支中的 `sky_color * (1 - result['acc'])`

### 12.4 真实视角与新视角训练损失

真实视角使用真实图像 `viewpoint_cam.original_image`，损失定义为：

$$
L_{real} =
(1-\lambda_{dssim})\lambda_{l1}L_1(I, I^*, M)
+\lambda_{dssim}(1-SSIM(I, I^*, M))
+\lambda_{lpips}LPIPS(I\odot M, I^*\odot M)
$$

对应代码为 `train.py`：

- `Ll1 = l1_loss(image, gt_image, mask)`
- `ssim_value = ssim(image, gt_image, mask=mask)`
- `lpips_value = lpips(image * mask, gt_image * mask)`
- `loss = (1.0 - lambda_dssim) * lambda_l1 * Ll1 + ...`

新视角使用扩散生成的 `viewpoint_cam.meta["diffusion_original_image"]` 作为伪 GT。代码只在图像下方 60% 区域计算损失：

$$
u = \lfloor 0.4H\rfloor,\quad
L_{novel}=\lambda_{novel}\left[
(1-\lambda_{novel\_dssim})\lambda_{novel\_l1}L_1(I_{u:H}, \hat I_{u:H}, M_{u:H})
+\lambda_{novel\_dssim}(1-SSIM)
+\lambda_{novel\_lpips}LPIPS
\right]
$$

对应代码为：

- `upper = int(mask.shape[-2] * 0.4)`
- `image_loss = image[:, upper:, :]`
- `gt_image_loss = gt_image[:, upper:, :]`
- `loss = loss * optim_args.lambda_novel`

真实视角还包含若干正则：

| 正则 | 公式形式 | 代码变量 |
|------|----------|----------|
| sky loss | sky 区域 $-\log(1-A)$，非 sky 区域使用 alpha entropy | `sky_loss`, `sky_mask`, `acc` |
| object acc | object bound 内用 entropy，bound 外用 $-\log(1-A_{obj})$ | `obj_acc_loss`, `obj_bound`, `acc_obj` |
| LiDAR depth | 取 $\lvert D-D_{lidar}\rvert$ 最小的 95% 点求均值 | `depth_error`, `torch.topk(..., largest=False)` |
| scale flatten | $\min(s)+\frac{s_1^2+s_2^2}{s_1s_2}-2$ | `scaling_reg_loss`, `scaling.topk(2)` |

### 12.5 扩散训练损失与条件输入

`video_diffusion/vwm/modules/diffusionmodules/loss.py::StandardDiffusionLoss` 使用 EDM 风格噪声级别：

$$
x_\sigma = x + \sigma\epsilon
$$

对应代码：

- `sigmas = self.sigma_sampler(input.shape[0]).to(input)`
- `noise = torch.randn_like(input)`
- `noised_input = input + noise * sigmas_bc`

Denoiser 预测 clean latent，损失按 sigma weighting 加权：

$$
L_{diff}=\mathbb{E}\left[w(\sigma)\lVert \hat{x}_0-x\rVert_p\right]
$$

对应代码：

- `model_output = denoiser(network, noised_input, sigmas, cond, cond_mask)`
- `w = append_dims(self.loss_weighting(sigmas), input.ndim)`
- L2 分支：`(w * (predict - target) ** 2)`
- L1 分支：`(w * (predict - target).abs())`

LiDAR guidance 在推理封装中先经过 VAE 编码：

- `guidance = model.encode_first_stage(value_dict['guide_frames'])`
- conditional 分支使用 `guidance_c["scale"] = 1`
- unconditional 分支使用 `guidance_uc["scale"] = 0`

这对应条件视频扩散中的“有 LiDAR 条件”和“无 LiDAR 条件”两支，用于 classifier-free guidance。

### 12.6 训练外引导采样

`EulerEDMSamplerSDS` 在 `cond` 中存在 `sample_guidance` 时，不从纯噪声开始，而从当前 Gaussian render latent 附近开始：

$$
N_{infer}=\lfloor N\cdot scale\rfloor
$$

$$
start=N-N_{infer}
$$

$$
z_{start}=z_{render}+\sigma_{start}\epsilon
$$

对应代码为 `video_diffusion/vwm/modules/diffusionmodules/sampling.py::EulerEDMSamplerSDS.__call__()`：

- `num_inference_steps = int(self.num_steps * scale)`
- `start_step = self.num_steps - num_inference_steps`
- `render_latents = cond['sample_guidance']['input']`
- `x = render_latents + x * start_sigma`

条件帧替换使用 mask 混合：

$$
x \leftarrow x(1-M_{cond})+z_{cond}M_{cond}
$$

对应代码：

- `cond_mask[cond_indices] = 1`
- `x = x * append_dims(1 - cond_mask, x.ndim) + cond_frame * append_dims(cond_mask, cond_frame.ndim)`

需要注意的是，`sampler_step()` 中更细粒度的 mask blending 分支仍处于注释状态；实际生效的是“以 render latent 作为带噪初值”的训练外引导。

## 13. 工程调试与扩展建议

StreetCrafter 的代码结构把数据预处理、条件视频生成和 3DGS 蒸馏拆成多个相对独立的入口。二次开发时，较稳妥的排查顺序是先确认数据产物，再确认相机与条件图路径，最后进入训练循环和损失项。

### 13.1 数据侧检查

- 原始 Waymo/PandaSet 数据应先被转换为统一场景目录，场景目录中至少需要包含图像、相机内外参、ego pose、tracklet 和 LiDAR 产物。
- LiDAR 条件图与 mask 的文件命名必须和 camera metadata 中的 `guidance_rgb_path`、`guidance_mask_path` 一致；新轨迹 shift 会改变条件图目录名，例如 `color_render_shift_2.00`。
- 新视角训练依赖扩散伪监督。如果 `camera.meta["diffusion_original_image"]` 不存在，新视角 camera 不应被当作有效伪 GT 使用。

### 13.2 模型侧检查

- 若 3DGS 在真实视角上无法收敛，应优先检查 PLY 初始化、相机坐标系、mask、sky mask 和 LiDAR depth，而不是先调扩散模型。
- 若新视角出现结构漂移，应检查 lane shift 方向、`LANE_SHIFT_SIGN`、actor `skip_camera` 判断和新轨迹 LiDAR 条件是否同步生成。
- 若扩散结果稳定但蒸馏结果变差，应关注 `lambda_novel`、下方 60% 伪监督区域、LPIPS 权重和 `sample_scales` 的时间调度。

### 13.3 常见扩展点

- 新数据集接入通常需要实现 dataparser、相机列表构造、tracklet/actor metadata、LiDAR 条件渲染和 Dataset callback 注册。
- 多相机新视角需要同时扩展 novel camera 生成逻辑、条件图路径规则、扩散窗口组织和保存逻辑。
- 更强的扩散引导可以从 `EulerEDMSamplerSDS` 的 mask blending 注释分支、`training_free_guidance` 输入和 `masked_guidance` 配置入手。
- 更复杂的动态目标建模可以从 actor local Gaussian 初始化、`ActorPose` 优化和 deformable actor 的时间特征表示入手。

## 14. 实现边界与注意事项

1. `street_gaussian/datasets/dataset.py` 只注册了 Waymo 读取函数。仓库中有 PandaSet 的 processor 和 video diffusion dataset，但顶层 3DGS Dataset 回调未注册 PandaSet。
2. 扩散采样中的 mask blending 逻辑目前在 `EulerEDMSamplerSDS.sampler_step()` 中是注释状态；实际生效的是以 Gaussian render latent 作为带噪初值的训练外引导。
3. 新视角扩散和新视角训练主要面向前视相机 `cam == 0`。
4. 新视角伪监督只使用图像下方 60% 区域计算损失，上方 40% 不参与该损失。
5. `include_sky` 与 `include_cube_map` 互斥；默认示例使用 cubemap sky。
6. 多数可选正则项由配置权重控制，权重为 0 时不会参与优化。
7. 训练恢复时，如果 `cfg.trained_model_dir` 非空，会加载最新或指定 checkpoint；如果 `cfg.resume=False`，输出目录中的记录和 checkpoint 会被清理。
