# Instinct Onboard - Unitree G1 部署指南

`31dof parkour` 相关部署代码已移除。当前仓库保留的入口主要是追踪和工具脚本。

## 当前保留的脚本

| 脚本 | 说明 |
|------|------|
| `scripts/g1_perceptive_track.py` | 深度相机 + 动作序列追踪 |
| `scripts/g1_track.py` | 无相机动作序列追踪 |
| `scripts/depth_latent_publisher.py` | 深度隐特征发布 |
| `scripts/rs_cam_test.py` | RealSense 相机测试 |

## 基本环境

- Ubuntu 22.04
- ROS2 Humble
- Python 3.8+
- Unitree ROS2 相关消息包

安装依赖：

```bash
cd hmv_test
pip install -e .
```

## 模型目录

模型目录通常包含：

```text
/path/to/model/
├── params/
│   ├── env.yaml
│   └── agent.yaml
└── exported/
    ├── actor.onnx
    ├── policy_normalizer.npz
    └── 0-depth_encoder.onnx
```

动作序列目录示例：

```text
/path/to/motions/
├── diveroll4-ziwen-0-retargeted.npz
├── kneelClimbStep1-x-0.1-ziwen-retargeted.npz
└── ...
```

## 常用命令

感知追踪：

```bash
python scripts/g1_perceptive_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions
```

基础追踪：

```bash
python scripts/g1_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions
```

相机测试：

```bash
python scripts/rs_cam_test.py
```

## 说明

- 如果需要真机运行，给脚本添加 `--nodryrun`
- 如果需要深度图或点云调试，使用对应脚本支持的 `--depth_vis`、`--pointcloud_vis`
- 如果需要远程调试，使用 `--debug`
