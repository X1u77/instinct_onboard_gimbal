# Scripts - 入口脚本说明

本目录包含当前保留的可执行入口。`31dof parkour` 相关脚本已删除。

> 注意：不要从其他模块导入本目录下的脚本文件，它们按独立入口设计。

## 当前脚本

### g1_perceptive_track.py

深度相机 + 动作序列追踪。

```bash
python scripts/g1_perceptive_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions \
    [--walk_logdir /path/to/walk/model] \
    [--nodryrun] [--depth_vis] [--pointcloud_vis] [--motion_vis]
```

### g1_track.py

无相机动作序列追踪。

```bash
python scripts/g1_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions \
    [--nodryrun] [--motion_vis]
```

### rs_cam_test.py

RealSense 相机测试。

```bash
python scripts/rs_cam_test.py
```

### depth_latent_publisher.py

深度隐特征发布器。

```bash
python scripts/depth_latent_publisher.py \
    --logdir /path/to/model \
    --publish_frequency 10.0 \
    [--visualize_depth]
```

## 通用参数

| 参数 | 说明 |
|------|------|
| `--nodryrun` | 真机模式（默认 Dry-run） |
| `--debug` | 启用 debugpy 远程调试 |
