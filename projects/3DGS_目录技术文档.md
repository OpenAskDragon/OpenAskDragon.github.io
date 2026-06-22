---
title: 3DGS 目录技术文档
date: 2026-06-22
tags:
  - 3DGS
  - Gaussian Splatting
  - Visual Localization
  - RaDe-GS
  - SplatLoc
  - gsplat
gitalk: false
---

# 3DGS 目录技术文档

本文档面向当前目录 `E:\绩效考核\分享\one_shot_targetless_based_camera_LiDAR_calib_algo\3DGS`，依据本地文件树、各子项目 README、环境文件、配置文件、入口脚本和已有实验总结整理。文档目标是客观描述目录内各 3D Gaussian Splatting 相关项目的职责、运行入口、依赖关系、实验结论和后续使用注意事项。

需要注意：当前目录未初始化 CodeGraph 索引，因此本文没有使用 AST 知识图谱结果；结构分析来自本地文件扫描、Python AST 摘要和项目说明文件。目录内包含多个上游开源项目或其派生版本，本文只描述当前本地副本可见内容，不代替各项目官方文档。

## 1. 总览

当前 `3DGS` 目录不是单一工程，而是一个 3D Gaussian Splatting 研究与实验工作区，包含以下主要部分：

| 路径 | 主要角色 | 当前可见特征 |
|---|---|---|
| `gaussian-splatting/` | GraphDECO 原始 3DGS 参考实现 | 训练、渲染、指标计算、COLMAP 数据转换、SIBR viewer 相关资源 |
| `RaDe-GS/` | Rasterizing Depth in Gaussian Splatting | 当前 L2 桥梁数据实验实际使用的训练主体，支持深度、法线、多视图正则和 mesh extraction |
| `gsplat/` | Nerfstudio 团队维护的 CUDA Gaussian rasterization 库 | Python/CUDA rasterization、训练策略、压缩、LiDAR rasterization、实验性 inference rendering |
| `gsplatloc/` | GSplatLoc 视觉定位系统 | 将关键点描述子嵌入 3DGS，用于 2D-3D 匹配、PnP 初值和位姿细化 |
| `SplatLoc/` | SplatLoc AR 视觉定位系统 | 训练 3D feature decoder 与 3D Gaussian，评估渲染、定位和 landmark selection |
| `nerfstudio/` | Nerfstudio 框架 | NeRF/3DGS 通用训练框架、viewer、数据处理和插件基础 |
| `splatfacto-w/` | Nerfstudio 的 in-the-wild Gaussian Splatting 插件 | 面向 PhotoTourism/野外采集的 appearance/transient 建模插件 |
| `L2_3DGS_EXPERIMENT_SUMMARY.md` | 本地 L2 桥梁数据实验总结 | 记录了 RaDe-GS + LAS 初始化 + gsplat 评估的最佳配置和指标 |

文件规模上，当前目录包含大量 C++/CUDA/OpenGL viewer 资源、HTML 文档和 Python 训练脚本。按文件扩展名粗略统计，`.py` 文件约 608 个，`.cpp` 文件约 875 个，`.hpp` 文件约 1066 个，`.cu` 文件约 56 个，`.md` 文件约 111 个。需要区分核心训练代码和第三方 viewer/submodule 代码。

## 2. 3DGS 基础概念

3D Gaussian Splatting 将场景表示为一组可优化的三维高斯 primitive。每个 Gaussian 通常包含：

- 三维中心位置 `xyz`
- 不透明度 `opacity`
- 各向异性尺度 `scale`
- 旋转 `rotation`
- 颜色或球谐系数 `SH`
- 在扩展系统中还可能包含深度、法线、语义特征、关键点描述子或 appearance embedding

渲染时，三维 Gaussian 会投影到图像平面，形成屏幕空间的二维 footprint，再通过可微 rasterization 累积颜色、深度或特征。因此屏幕上看到的是二维 splat footprint，但模型本体仍是三维 Gaussian。

原始 3DGS 的典型训练循环包括：

1. 从 COLMAP/SfM 或其他点云初始化 Gaussian。
2. 根据训练相机渲染当前视角。
3. 使用 RGB L1、SSIM 等重建损失优化位置、颜色、尺度、旋转和不透明度。
4. 周期性进行 densification、split、clone、prune 和 opacity reset。
5. 保存 `point_cloud/iteration_xxx/point_cloud.ply` 作为训练结果。

当前目录中的 RaDe-GS、SplatLoc、GSplatLoc 和 Splatfacto-W 都可以看作在这个基础范式上添加深度/几何正则、视觉定位特征或野外场景鲁棒建模。

## 3. 子项目详解

### 3.1 `gaussian-splatting/`

这是 GraphDECO/Inria 的 3D Gaussian Splatting 参考实现。本地 README 说明该项目包含四类组件：

- PyTorch 优化器：从 SfM 输入训练 3D Gaussian 模型。
- 网络 viewer：连接训练过程并实时查看优化状态。
- OpenGL/SIBR 实时 viewer：加载训练完成的模型并实时渲染。
- `convert.py`：将自定义图像转换成优化可用的 COLMAP/SfM 数据结构。

核心入口：

| 文件 | 功能 |
|---|---|
| `train.py` | 训练 3DGS 模型 |
| `render.py` | 渲染 train/test 视角 |
| `metrics.py` | 计算 PSNR、SSIM、LPIPS 等指标 |
| `full_eval.py` | 批量复现实验评估流程 |
| `convert.py` | 调用 COLMAP/ImageMagick 生成标准数据结构 |
| `render_novel_poses.py` | 本地扩展的 novel pose 渲染工具 |

核心模块：

| 模块 | 职责 |
|---|---|
| `scene/gaussian_model.py` | Gaussian 参数、优化器、densification/prune、PLY 存取 |
| `scene/dataset_readers.py` | COLMAP 和 NeRF synthetic 数据读取 |
| `scene/cameras.py` | Camera/MiniCam 数据结构 |
| `gaussian_renderer/__init__.py` | 可微 Gaussian rasterization 调用封装 |
| `arguments/__init__.py` | `ModelParams`、`PipelineParams`、`OptimizationParams` 参数组 |
| `utils/graphics_utils.py` | 相机投影、坐标变换、FoV/focal 转换 |
| `utils/loss_utils.py` | L1/L2/SSIM/Fused SSIM 等损失 |

标准训练命令：

```bash
python train.py -s <COLMAP_or_NeRF_dataset>
```

标准评估命令：

```bash
python train.py -s <dataset> --eval
python render.py -m <model_dir>
python metrics.py -m <model_dir>
```

数据结构要求：

```text
<dataset>
|-- images/
|-- sparse/
    |-- 0/
        |-- cameras.bin
        |-- images.bin
        |-- points3D.bin
```

本地环境文件 `environment.yml` 指定了 `python=3.7.13`、`pytorch=1.12.1`、`cudatoolkit=11.6`，并通过 pip 安装 `diff-gaussian-rasterization`、`simple-knn`、`fused-ssim` 等子模块。官方 README 同时提到较新 PyTorch/CUDA 也可能可用，但结果可重复性会受环境影响。

### 3.2 `RaDe-GS/`

RaDe-GS 全称为 Rasterizing Depth in Gaussian Splatting。README 显示其论文已被 ACM TOG 接收，并集成了 PGSR 的 multi-view regularization。当前 L2 实验总结明确指出，本机 L2 suspension bridge 数据训练入口使用的是 `3DGS/RaDe-GS/train.py`。

核心入口：

| 文件 | 功能 |
|---|---|
| `train.py` | 训练 RaDe-GS 模型 |
| `render.py` | novel view synthesis 渲染 |
| `metric.py` | 渲染指标评估 |
| `mesh_extract.py` | 通用 mesh extraction |
| `mesh_extract_tnt.py` | Tanks and Temples mesh extraction |
| `mesh_extract_tetrahedra.py` | Marching Tetrahedra 相关 mesh extraction |
| `evaluate_dtu_mesh.py` | DTU 几何评估 |
| `geometry_metric.py` | Objaverse 等几何指标 |
| `eval_tnt/run.py` | Tanks and Temples 几何评估流程 |

重要模块：

| 模块 | 职责 |
|---|---|
| `gaussian_renderer/__init__.py` | 除 RGB render 外，还提供 `integrate`、`sample_depth` 等深度相关接口 |
| `scene/gaussian_model.py` | Gaussian 模型与训练参数 |
| `scene/appearance_network.py` | decoupled appearance 网络 |
| `utils/graphics_utils.py` | 深度转法线、重投影、双线性采样等几何工具 |
| `utils/loss_utils.py` | RGB/SSIM/appearance/multi-view patch matching 损失 |
| `utils/tetmesh.py` | marching tetrahedra 支持 |

README 给出的依赖安装流程包括：

```bash
conda create -n radegs python=3.12
conda activate radegs
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu130
pip install -r requirements.txt
pip install submodules/diff-gaussian-rasterization --no-build-isolation
pip install submodules/warp-patch-ncc --no-build-isolation
pip install submodules/simple-knn/ --no-build-isolation
pip install git+https://github.com/rahul-goel/fused-ssim/ --no-build-isolation
conda install conda-forge::cgal
pip install submodules/tetra_triangulation/ --no-build-isolation
```

本地 `requirements.txt` 仅列出 Python 层依赖：`open3d`、`trimesh`、`scikit-image`、`opencv-python`、`plyfile`、`tqdm`。实际训练还依赖 CUDA 扩展。

典型命令：

```bash
# DTU
python train.py -s <path_to_dtu> -m <output_dir> -r 2 --use_decoupled_appearance 3
python mesh_extract.py -m <output_dir>
python evaluate_dtu_mesh.py -m <output_dir>

# Tanks and Temples
python train.py -s <path_to_preprocessed_tnt> -m <output_dir> -r 2 --use_decoupled_appearance 3
python mesh_extract_tnt.py -m <output_dir>
python eval_tnt/run.py --dataset-dir <gt_tnt> --traj-path <COLMAP_SfM.log> --ply-path <output_dir>/recon_post.ply --out-dir <output_dir>/mesh

# Novel view synthesis
python train.py -s <dataset> -m <output_dir> --eval
python render.py -m <output_dir>
python metrics.py -m <output_dir>
```

与原始 3DGS 相比，RaDe-GS 更偏重几何、深度、法线和 mesh 输出。对当前 L2 桥梁数据，它被用作大场景 3DGS 训练主体。

### 3.3 `gsplat/`

`gsplat` 是 Nerfstudio 团队维护的开源 CUDA Gaussian rasterization 库。README 描述其目标是提供更快、更省显存、功能更丰富的 Gaussian splatting Python 绑定。当前本地副本 README 包含 2026 年未发布更新，如 HiGS inference rendering、LiDAR rasterization、TorchScript-oriented camera operators、3DGUT 扩展等。

核心功能：

- CUDA 加速 3D/2D Gaussian rasterization。
- Python API `gsplat.rendering.rasterization(...)`。
- 多场景/多视角 batch 支持。
- Gaussian 压缩、PLY/splat 导出。
- DefaultStrategy、MCMCStrategy 等 densification/训练策略。
- LiDAR rasterization 和 `eval3d` 渲染路径。
- 实验性 `experimental.render` inference-only pipeline。

重要模块：

| 模块 | 职责 |
|---|---|
| `gsplat/rendering.py` | 主 rasterization API 和 render mode 判断 |
| `gsplat/cuda/_wrapper.py` | CUDA op 懒加载封装 |
| `gsplat/cuda/_torch_impl.py` | PyTorch fallback/参考实现 |
| `gsplat/cuda/_torch_impl_2dgs.py` | 2DGS 相关实现 |
| `gsplat/cuda/_torch_impl_lidar.py` | LiDAR rasterization 支持 |
| `gsplat/strategy/default.py` | 默认 densification 策略 |
| `gsplat/strategy/mcmc.py` | MCMC 风格 Gaussian 训练策略 |
| `gsplat/compression/` | PNG/KMeans/NPZ 等压缩实现 |
| `gsplat/exporter.py` | PLY/splat 文件导入导出 |

安装方式：

```bash
pip install gsplat
```

或源码安装：

```bash
pip install git+https://github.com/nerfstudio-project/gsplat.git
```

本地 `pyproject.toml` 显示构建依赖包括 `setuptools`、`wheel`、`ninja`、`torch`、`numpy`、`rich`。README 说明 PyPI 安装方式会在首次运行时 JIT 构建 CUDA 代码；源码安装则安装期构建。

在当前工作区中，`gsplat` 的直接价值是作为高性能真实 splat 渲染和评估工具。L2 实验总结中也明确提到当前评估使用 gsplat 真实 splat rasterization，而不是简单点云投影。

### 3.4 `gsplatloc/`

`gsplatloc` 对应论文 “GSplatLoc: Grounding Keypoint Descriptors into 3D Gaussian Splatting for Improved Visual Localization”。其目标是在 3DGS 中承载关键点描述子，改善视觉定位中的 2D-3D 匹配。

核心流程：

1. 准备 7Scenes 或 Cambridge Landmarks 数据。
2. 用 SfM 点云初始化 3DGS。
3. 训练带特征的 Gaussian 模型。
4. 使用 XFeat 等关键点/描述子从 query image 中提取 2D 特征。
5. 从 Gaussian feature field 中建立 2D-3D 对应关系。
6. 通过 PnP-RANSAC 得到 pose prior。
7. 使用 differentiable warping 进一步细化位姿。

核心入口：

| 文件 | 功能 |
|---|---|
| `train.py` | 训练带描述子/特征的 3DGS |
| `loc_inference.py` | 定位推理主流程，包含 pose prior estimation 和 pose refinement |
| `render.py` | 渲染模型输出 |
| `view.py` | viewer 入口 |
| `metrics.py` | 指标计算 |
| `datasets/setup_7scenes.py` | 7Scenes 数据准备 |
| `datasets/setup_cambridge.py` | Cambridge Landmarks 数据准备 |

关键模块：

| 模块 | 职责 |
|---|---|
| `scene/gaussian_model.py` | Gaussian 参数和特征承载 |
| `gaussian_renderer/__init__.py` | RGB/feature/depth 渲染与 edit 支持 |
| `utils/loc_utils.py` | pose error、2D-3D correspondence 查找 |
| `warping/warping_loss.py` | differentiable warp pose refinement loss |
| `models/networks.py` | MLP/CNN feature decoder |
| `encoders/XFeat/` | XFeat 关键点描述子相关代码 |

典型训练命令：

```bash
python train.py -s data/DATASET_NAME -m output/OUTPUT_NAME --iterations 7000
```

定位命令：

```bash
python loc_inference.py -m output/OUTPUT_NAME
```

本地 README 指出 `diff-gaussian-rasterization` 模块默认面向 64 维 XFeat descriptor；若要换用不同维度的 encoder，需要修改 CUDA rasterizer 中的 `NUM_SEMANTIC_CHANNELS` 并重新编译。

### 3.5 `SplatLoc/`

`SplatLoc` 对应 “SplatLoc: 3D Gaussian Splatting-based Visual Localization for Augmented Reality”。其方法是使用单目 RGB-D 帧重建场景 3D Gaussian，同时学习无偏 3D descriptor field，用于精确 2D-3D feature matching 和 6-DoF camera pose estimation。

核心入口：

| 文件 | 功能 |
|---|---|
| `train_decoder.py` | 训练 3D feature decoder |
| `train_gaussians.py` | 训练 3D Gaussian 模型 |
| `test.py` | 定位、渲染和 landmark selection 评估 |
| `replica.sh` | Replica 数据集批处理脚本 |
| `scenes12.sh` | 12-Scenes 数据集批处理脚本 |

预处理模块：

| 文件 | 功能 |
|---|---|
| `pre_process/extract_save_sp_feature.py` | SuperPoint feature map 和 keypoint score 提取 |
| `pre_process/gen_netvlad_retrieval.py` | NetVLAD 图像检索结果生成 |
| `pre_process/gen_3d_fusion_feature.py` | 3D feature volume/cloud 融合 |

视觉化模块：

| 文件 | 功能 |
|---|---|
| `visualizations/render_localization.py` | 用 Open3D 渲染定位过程 |
| `visualizations/render_localization_with_matches.py` | 显示带 2D-3D match 的定位过程 |

重要内部模块：

| 模块 | 职责 |
|---|---|
| `utils/dataset.py` | `ReplicaDataset`、`Scenes12Dataset` 和数据加载 |
| `utils/fusion_utils.py` | TSDFVolumeTorch、feature fusion、mesh/point cloud 写出 |
| `utils/eval_utils.py` | 渲染、pose、selection 评估 |
| `utils/selection.py` | 3D landmark selection |
| `utils/match_utils.py` | Hungarian matching |
| `models/decoders.py` | FeatureNet、FeatureDecoder |
| `models/encoding.py` | HashGrid/frequency/identity encoder 构建 |

README 给出的训练和评估方式：

```bash
# train the 3D feature decoder
CUDA_VISIBLE_DEVICES=1 python train_decoder.py --config ./configs/replica_nerf/$scene.yaml

# train the 3D gaussian model
CUDA_VISIBLE_DEVICES=1 python train_gaussians.py --config ./configs/replica_nerf/$scene.yaml

# eval localization and rendering
CUDA_VISIBLE_DEVICES=1 python test.py --config ./configs/replica_nerf/$scene.yaml --eval_pose --eval_rendering

# eval 3D landmark selection
CUDA_VISIBLE_DEVICES=1 python test.py --config ./configs/replica_nerf/$scene.yaml --eval_selection --landmark_num 5000
```

配置文件中可见的关键默认值：

| 配置项 | Replica 默认值 | 12-Scenes 默认值 |
|---|---:|---:|
| 图像宽高 | 640 x 480 | 640 x 480 |
| `final_dim` | 256 | 256 |
| `opt_params.iterations` | 30000 | 30000 |
| `densification_interval` | 100 | 100 |
| `densify_from_iter` | 500 | 500 |
| `densify_until_iter` | 15000 | 15000 |
| `sh_degree` | 0 | 0 |
| `data_device` | cuda | cuda |

本地 `environment.yml` 使用 `python=3.7.13`、`pytorch=1.12.1`、`cudatoolkit=11.6`，并依赖 `open3d==0.17.0`、`evo==1.11.0`、`pycolmap`、`torchmetrics`、`lpips`、`wandb`、`PyOpenGL` 等。

### 3.6 `nerfstudio/`

`nerfstudio` 是通用 NeRF/3DGS 框架。README 表明它提供：

- 模块化训练 pipeline。
- Web viewer。
- 数据处理命令 `ns-process-data`。
- 训练命令 `ns-train`。
- 导出命令 `ns-export`。
- 视频渲染命令 `ns-render`。
- 插件注册机制。

典型安装：

```bash
conda create --name nerfstudio -y python=3.8
conda activate nerfstudio
pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118
conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit
pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
pip install -e .
```

典型训练：

```bash
ns-download-data nerfstudio --capture-name=poster
ns-train nerfacto --data data/nerfstudio/poster
```

在当前目录中，`nerfstudio` 主要为 `splatfacto-w` 插件提供运行基础，也可以作为其他 3DGS/NeRF 方法的对照框架。

### 3.7 `splatfacto-w/`

`splatfacto-w` 是 Nerfstudio 的 Gaussian Splatting for in-the-wild captures 插件。本地 `pyproject.toml` 显示其依赖 `nerfstudio >= 1.1.0`，并注册了两个 Nerfstudio method config：

- `splatfacto-w`
- `splatfacto-w-light`

还注册了一个 dataparser：

- `splatfactow_dataparser`

关键模块：

| 模块 | 职责 |
|---|---|
| `splatfactow/splatfactow_model.py` | 主模型配置和模型实现 |
| `splatfactow/splatfactow_field.py` | background field 和 SplatfactoW field |
| `splatfactow/splatfactow_datamanager.py` | 数据管理、图像 undistort |
| `splatfactow/nerfw_dataparser.py` | NeRF-W/PhotoTourism 数据解析 |
| `export_script.py` | 导出 PLY 文件 |

安装注册：

```bash
conda activate nerfstudio
cd splatfacto-w/
pip install -e .
ns-install-cli
```

PhotoTourism/NeRF-W 风格训练：

```bash
ns-download-data phototourism --capture-name <capture_name>
ns-train splatfacto-w --data <PATH> nerf-w-data-parser-config --data_name <trevi|sacre|brandenburg>
```

通用轻量版本：

```bash
ns-train splatfacto-w-light [OPTIONS] --data <PATH> <dataparser>
```

README 中建议的增强选项：

- `--pipeline.model.enable_bg_model True`
- `--pipeline.model.enable_alpha_loss True`
- `--pipeline.model.enable_robust_mask True`

这些选项分别用于背景建模、sky/alpha 约束和 transient object 鲁棒处理，适合曝光变化、动态对象、遮挡和非受控采集数据。

## 4. 当前 L2 桥梁数据实验结论

根目录的 `L2_3DGS_EXPERIMENT_SUMMARY.md` 记录了本机 L2 suspension bridge 数据的 3DGS 实验。该实验是当前目录中最具体、最可复用的实证材料。

### 4.1 数据与任务

L2 图像数据：

```text
data/3DGS/L2_roma_mixed/images
```

已记录特点：

- 共 35 张训练图像，编号 `000000` 到 `000034`。
- 原始分辨率为 `5280 x 3956`。
- RaDe-GS 训练时检测到宽度超过 1.6K，会自动缩放到约 1.6K 宽。
- 采集路径沿桥方向形成航线，不是环绕式多角度采集。
- 相机高度变化很小，基本处于同一飞行平面。
- 水面、车辆、船只、吊索细线、桥塔遮挡和局部时序差异会限制静态 3DGS 重建质量。

点云初始化：

```text
data/suspension_bridge.las
data/3DGS/L2_roma_mixed_las_init_1p5m
```

关键结论是：使用完整三维 LAS 点云初始化是 L2 数据训练出可用 3DGS 的前提。初始化点数为 1,500,000。早期未使用完整 LAS 初始化时，模型质量明显偏差，viewer 中容易出现黑屏或很差的 splat 分布。

### 4.2 最佳模型

当前实验总结给出的最优模型：

```text
outputs/l2_3dgs_rade_las_init_1p5m_mv200_densify8k_int1000_ncc015_14000/point_cloud/iteration_8000/point_cloud.ply
```

核心指标：

| 评估集 | PSNR | SSIM | MAE |
|---|---:|---:|---:|
| 5 个抽样训练视角 `0,8,16,24,32` | 23.0073 | 0.9080 | 0.04253 |
| 全部 35 个训练视角 `0-34` | 23.0417 | 0.9097 | 0.04246 |

最佳配置：

| 参数 | 值 |
|---|---:|
| LAS init points | 1.5M |
| `multi_view_max_dis` | 200 |
| `multi_view_max_angle` | 90 |
| `densify_from_iter` | 1500 |
| `densify_until_iter` | 8000 |
| `densification_interval` | 1000 |
| `densify_grad_threshold` | 0.001 |
| `opacity_reset_interval` | 100000 |
| `regularization_from_iter` | 8000 |
| `lambda_multi_view_ncc` | 0.15 |
| best iteration | 8000 |
| Gaussian count | 1,246,137 |

### 4.3 调参结论

实验总结中比较了多个设置：

| 配置 | 关键特点 | 结果概述 |
|---|---|---|
| 早期模型 | 未正确使用完整 LAS 初始化或配置不适配 | PSNR 约 16.23 dB，质量较差 |
| LAS 1.5M + no densify | 关闭 densification | 20k iteration 约 20.95 PSNR，上限有限 |
| visible LAS init | 只保留训练相机可见点 | 7000 iteration 约 20.01 PSNR，未显著优于原始 LAS 初始化 |
| densify3k / interval 250 | 早期有限 densify，频率较高 | 14k iteration 约 21.65 PSNR |
| densify8k / interval 1000 | 大场景稀疏 densify | 8000 iteration 达到约 23.04 PSNR，是当前最佳 |

主要结论：

- 对 L2 这种大场景，多图训练时 densification/prune 不宜过于频繁。
- `densification_interval=1000` 明显优于 `250`。
- `densify_until_iter=8000` 与 `regularization_from_iter=8000` 的组合在当前实验中较优。
- 最佳停止点是 `8000 iteration`，不是 14000 或 20000。
- 继续训练会拉低部分视角质量，尤其是水面和桥塔附近视角。

### 4.4 查看与评估

实验总结中记录的 viewer 启动脚本：

```text
tools/run_l2_best_3dgs_viewer.cmd
```

启动后访问：

```text
http://localhost:8082
```

评估方式使用 gsplat 真实 splat 渲染，而不是点云投影。5 视角评估命令模式：

```powershell
D:\miniforge\envs\da3\python.exe tools\evaluate_gsplat_train_views.py `
  --ply outputs\l2_3dgs_rade_las_init_1p5m_mv200_densify8k_int1000_ncc015_14000\point_cloud\iteration_8000\point_cloud.ply `
  --cameras outputs\l2_3dgs_rade_las_init_1p5m_mv200_densify8k_int1000_ncc015_14000\cameras.json `
  --images data\3DGS\L2_roma_mixed\images `
  --out outputs\gsplat_quality_eval_l2_las_init_mv200_densify8k_int1000_ncc015_8000 `
  --ids 0,8,16,24,32 `
  --width 1280 `
  --height 960
```

注意：上述 `tools/` 路径不在当前 `3DGS` 目录文件扫描结果中，可能位于上级工作区或另一路径。复现实验时应先确认 `tools` 目录存在。

## 5. 典型工作流

### 5.1 使用 COLMAP 图像训练标准 3DGS

适用项目：`gaussian-splatting/`

```bash
cd gaussian-splatting
python convert.py -s <image_project_dir> --resize
python train.py -s <image_project_dir> -m <output_dir>
python render.py -m <output_dir>
python metrics.py -m <output_dir>
```

适用前提：

- 输入数据已完成 COLMAP 或可通过 `convert.py` 调用 COLMAP。
- 相机模型可转换为 SIMPLE_PINHOLE 或 PINHOLE。
- CUDA、PyTorch 和 rasterizer 扩展可正常编译。

### 5.2 使用 LAS/点云初始化大场景 3DGS

适用项目：当前 L2 实验使用 `RaDe-GS/`。

推荐原则来自本地实验：

- 优先保证点云初始化与相机坐标系一致。
- 对大场景使用较慢 densification/prune。
- 不要过早 prune 尚未收敛的 Gaussian。
- 对水面、车辆、船只等静态模型难以解释的区域考虑 mask 或降低权重。
- 使用真实 splat renderer 评估，不用简单投影替代。

### 5.3 训练 3DGS 视觉定位系统

可选项目：

- `SplatLoc/`：面向 RGB-D/AR 场景，重点是 3D descriptor field 和 2D-3D matching。
- `gsplatloc/`：面向关键点描述子 grounding，重点是 XFeat/feature Gaussian/PnP/warp refinement。

SplatLoc 典型流程：

```bash
cd SplatLoc
conda env create -f environment.yml
conda activate splatloc
pip install git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
cd submodules/diff-gaussian-rasterization
pip install -e .

cd ../..
python train_decoder.py --config ./configs/replica_nerf/<scene>.yaml
python train_gaussians.py --config ./configs/replica_nerf/<scene>.yaml
python test.py --config ./configs/replica_nerf/<scene>.yaml --eval_pose --eval_rendering
```

GSplatLoc 典型流程：

```bash
cd gsplatloc
conda create --name gsplatloc python=3.10
conda activate gsplatloc
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install -r requirements.txt
pip install submodules/diff-gaussian-rasterization
pip install submodules/simple-knn

python train.py -s data/DATASET_NAME -m output/OUTPUT_NAME --iterations 7000
python loc_inference.py -m output/OUTPUT_NAME
```

### 5.4 使用 Nerfstudio 插件训练野外 3DGS

适用项目：`nerfstudio/` + `splatfacto-w/`

```bash
cd nerfstudio
pip install -e .

cd ../splatfacto-w
pip install -e .
ns-install-cli

ns-train splatfacto-w-light --data <PATH> <dataparser>
```

该路线适合图像曝光变化、动态行人/车辆、天空背景和非受控采集条件，但与当前 L2 实验的 RaDe-GS 路线不同。

## 6. 依赖与环境注意事项

### 6.1 CUDA 扩展

目录内多个项目依赖 CUDA/C++ 扩展：

- `diff-gaussian-rasterization`
- `simple-knn`
- `fused-ssim`
- `warp-patch-ncc`
- `tetra_triangulation`
- `gsplat` runtime extension
- `tiny-cuda-nn` bindings

这些扩展对 PyTorch、CUDA Toolkit、MSVC/g++、Python 版本非常敏感。在 Windows 下，常见问题是找不到 `cl.exe` 或 CUDA/PyTorch ABI 不匹配。

### 6.2 Python/PyTorch 版本不统一

当前目录中各项目推荐环境并不一致：

| 项目 | Python | PyTorch/CUDA 建议 |
|---|---|---|
| `gaussian-splatting` | 3.7.13 | PyTorch 1.12.1 + CUDA 11.6 |
| `SplatLoc` | 3.7.13 | PyTorch 1.12.1 + CUDA 11.6 |
| `gsplatloc` | 3.10 | README 示例为 PyTorch cu118 |
| `RaDe-GS` | 3.12 | README 示例为 PyTorch cu130 |
| `nerfstudio` | >=3.8 | README 示例为 PyTorch 2.1.2 + CUDA 11.8 |
| `splatfacto-w` | 由 nerfstudio 决定 | `nerfstudio >= 1.1.0` |

因此不建议把所有项目强行安装进同一个 conda 环境。更稳妥的方式是按项目创建独立环境。

### 6.3 Windows 注意事项

当前 L2 实验总结中提到 Windows/Torch checkpoint 兼容修复：

```python
torch.load(checkpoint, weights_only=False)
```

位置记录为：

```text
3DGS/RaDe-GS/train.py
```

同时，viewer 在 Windows 下启动需要 Visual Studio 编译器环境，否则 gsplat runtime extension 可能找不到 `cl`。如果要复现 viewer，应确认：

- conda 环境激活正确。
- Visual Studio Build Tools 已安装。
- `cl.exe` 在当前终端 PATH 中可见。
- PyTorch CUDA runtime 与本机驱动兼容。
- 端口 `8082` 或训练 viewer 端口未被占用。

## 7. 质量评估指标

当前目录中常见指标包括：

| 指标 | 含义 | 注意事项 |
|---|---|---|
| PSNR | 基于 MSE 的像素误差指标 | 对曝光、错位、动态物体非常敏感 |
| SSIM | 结构相似性 | 更能反映局部结构，但仍受 mask/分辨率影响 |
| LPIPS | 感知相似性 | 依赖网络特征，适合感知质量对比 |
| MAE | 平均绝对误差 | 直观但不区分结构与颜色来源 |
| Pose error | 定位旋转/平移误差 | SplatLoc/GSplatLoc 更关注 |
| F-score / Chamfer-like | mesh 几何评估 | RaDe-GS/DTU/TnT 相关 |

L2 实验中已经指出，PSNR 95 dB 对真实大场景 3DGS 几乎等价于逐像素完全一致，不是合理目标。对有水面、动态目标、相机-LiDAR 标定误差和单航线视角的大场景，当前 `23.04 dB / 0.91 SSIM` 属于可解释结果；后续优化可现实地争取 24-26 dB，而不是追求 95 dB。

## 8. 当前目录的主要风险与边界

1. 多项目混合，环境互相不兼容。需要按子项目隔离 conda 环境。
2. 多个上游项目包含大量第三方 submodule、viewer、HTML 文档和编译产物，不能将文件数量直接理解为核心业务代码规模。
3. L2 实验依赖的 `tools/`、`data/`、`outputs/` 路径可能位于当前目录之外或未纳入当前扫描，需要复现前确认。
4. 大场景 3DGS 对坐标系、点云初始化、相机标定非常敏感。
5. 水面、车辆、船只等动态/反射对象不适合直接由静态 3DGS 完全解释。
6. 训练分辨率自动缩放会损失部分高频结构，例如桥索、栏杆和路面标线。
7. Densification 不是越频繁越好。当前 L2 实验表明大场景中 `interval=250` 反而不如 `interval=1000`。
8. Viewer 黑屏不一定表示模型无效，也可能是初始相机位置不在训练相机附近、路径含中文/空格、端口或编译环境问题。

## 9. 后续优化建议

面向当前 L2 桥梁实验，优先级建议如下：

1. 对车辆、船只等动态目标增加 mask。
2. 对水面区域增加 mask 或降低损失权重，避免水面反射主导优化。
3. 做曝光、白平衡和色彩一致化预处理。
4. 在 `densification_interval=1000` 附近做小范围网格实验，而不是回到高频 densification。
5. 测试 `regularization_from_iter=8500/9000` 和 `lambda_multi_view_ncc=0.05/0.1`。
6. 在显存允许时测试更高训练分辨率，但 RTX 3060 12GB 可能需要更保守的 batch、图像缩放或分块策略。
7. 如果可以补采数据，优先增加侧向、环绕和高度变化视角，这通常比单纯延长训练更有效。
8. 固化评估脚本，将 5-view 和 all-35 评估命令、输出路径和指标表自动化，减少手工比较误差。

## 10. 建议的目录使用方式

如果目标是继续 L2 大场景重建：

- 主要工作目录：`RaDe-GS/`
- 主要模型输出：`outputs/l2_3dgs_*`
- 评估工具：`gsplat` 真实 splat 渲染
- 关键数据：`data/3DGS/L2_roma_mixed*` 和 `data/suspension_bridge.las`

如果目标是视觉定位研究：

- RGB-D/AR 定位路线：`SplatLoc/`
- XFeat/关键点描述子路线：`gsplatloc/`

如果目标是通用 3DGS 基线或 COLMAP 图像训练：

- 使用 `gaussian-splatting/`

如果目标是 Nerfstudio 生态或野外照片集：

- 使用 `nerfstudio/` + `splatfacto-w/`

如果目标是高性能 rasterization、评估或二次开发：

- 使用 `gsplat/`

## 11. 快速命令索引

```bash
# 原始 3DGS 训练
cd gaussian-splatting
python train.py -s <dataset> -m <output_dir>

# 原始 3DGS 渲染与指标
python render.py -m <output_dir>
python metrics.py -m <output_dir>

# RaDe-GS 训练
cd RaDe-GS
python train.py -s <dataset> -m <output_dir> --eval

# RaDe-GS 渲染与评估
python render.py -m <output_dir>
python metric.py -m <output_dir>

# RaDe-GS mesh extraction
python mesh_extract.py -m <output_dir>

# SplatLoc 训练与评估
cd SplatLoc
python train_decoder.py --config ./configs/replica_nerf/<scene>.yaml
python train_gaussians.py --config ./configs/replica_nerf/<scene>.yaml
python test.py --config ./configs/replica_nerf/<scene>.yaml --eval_pose --eval_rendering

# GSplatLoc 训练与定位
cd gsplatloc
python train.py -s data/DATASET_NAME -m output/OUTPUT_NAME --iterations 7000
python loc_inference.py -m output/OUTPUT_NAME

# Nerfstudio 训练
cd nerfstudio
ns-train nerfacto --data <data_path>

# Splatfacto-W 插件训练
cd splatfacto-w
ns-train splatfacto-w-light --data <PATH> <dataparser>
```

## 12. 结论

当前 `3DGS` 目录是一个多项目 3D Gaussian Splatting 工作区，既包含原始 3DGS 基线，也包含几何增强、定位增强、高性能 rasterization 和 Nerfstudio 插件路线。对当前已有 L2 suspension bridge 实验而言，最有实证价值的路线是 `RaDe-GS` 训练、完整 LAS 点云初始化、稀疏 densification、`gsplat` 真实 splat 评估。

当前最佳 L2 模型为：

```text
outputs/l2_3dgs_rade_las_init_1p5m_mv200_densify8k_int1000_ncc015_14000/point_cloud/iteration_8000/point_cloud.ply
```

其全 35 训练视角评估为：

```text
PSNR = 23.0417
SSIM = 0.9097
MAE  = 0.04246
```

从现有实验看，继续优化的关键不是单纯增加 iteration，而是处理动态/水面区域、改善颜色一致性、控制大场景 densification/prune 时机，以及在数据层面增加更丰富的视角覆盖。
