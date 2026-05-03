# Scripts - 入口脚本说明

本目录包含所有可独立运行的入口脚本，每个脚本对应一种机器人的运行模式。

> **注意：请勿从其他模块导入本目录下的 Python 文件。这些脚本设计为独立运行。**

---

## 快速参考

```bash
# 1. 安装依赖后，进入目录
cd instinct_onboard_gimbal

# 2. Dry-run 测试（不连接机器人）
python scripts/g1_parkour.py \
    --logdir /path/to/model \
    --standdir /path/to/stand_model

# 3. 真机运行
python scripts/g1_parkour.py \
    --logdir /path/to/model \
    --standdir /path/to/stand_model \
    --nodryrun
```

---

## 脚本列表

### g1_parkour.py

跑酷模式（Parkour）

- 深度相机感知（RealSense D435）
- 障碍物穿越导航
- 可选：头部云台控制（UART 舵机）
- 遥操作：前进/后退、左右移、转弯

```bash
python scripts/g1_parkour.py \
    --logdir /path/to/parkour/model \
    --standdir /path/to/stand/model \
    [--nodryrun] [--gimbal] [--depth_vis] [--pointcloud_vis]
```

### g1_perceptive_track.py

感知追踪模式（Perceptive Tracking）

- 深度相机感知
- 动作序列播放（.npz 格式）
- 遥操作加载指定动作序列
- 支持与行走模式切换

```bash
python scripts/g1_perceptive_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions \
    [--walk_logdir /path/to/walk/model] \
    [--nodryrun] [--depth_vis] [--pointcloud_vis] [--motion_vis]
```

### g1_track.py

基础追踪模式（Basic Tracking）

- 无相机
- 动作序列播放
- 最简配置，适合调试

```bash
python scripts/g1_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions \
    [--nodryrun] [--motion_vis]
```

### g1_shadowing.py

影子跟随模式（Shadowing）

- 无固定动作序列
- 通过 ROS 话题实时接收动作参考
- 支持关节位置、末端位置、力矩等多种控制模式

```bash
python scripts/g1_shadowing.py \
    --logdir /path/to/shadowing/model \
    [--nodryrun]
```

### rs_cam_test.py

RealSense 相机测试

- 发布深度图和点云话题
- 用于验证相机连接和参数

```bash
python scripts/rs_cam_test.py
```

### depth_latent_publisher.py

深度隐特征发布器

- 将 RealSense 深度图编码为隐特征
- 发布到 ROS 话题供其他节点使用

```bash
python scripts/depth_latent_publisher.py \
    --logdir /path/to/model \
    --publish_frequency 10.0 \
    [--visualize_depth]
```

---

## 通用参数

所有脚本支持以下通用参数：

| 参数 | 说明 |
|------|------|
| `--nodryrun` | 真机模式（默认 Dry-run） |
| `--debug` | 启用 debugpy 远程调试（0.0.0.0:6789） |

---

## 状态机

所有包含 Agent 的脚本遵循相同的状态机逻辑：

```
启动 → ColdStart（自动）
              ↓ (完成后)
         Stand（按 R1）
              ↓ (按 L1)
         业务模式（Parkour / Tracking / Walk）
              ↓ (按 R1)
         Stand
```

**紧急停机：任何时刻按 R2 或 L2 触发急停。**
