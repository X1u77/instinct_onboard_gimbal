# G1 Parkour 真机部署

Unitree G1 31-DOF 跑酷策略 + RealSense D435 深度感知 + 头部云台舵机

## 快速启动

```bash
# 1. 安装依赖
pip install -e instinct_onboard_gimbal/

# 2. Dry-run（安全测试，不发命令给机器人）
python scripts/g1_parkour.py \
    --logdir /path/to/parkour/model \
    --standdir /path/to/stand/model \
    --gimbal

# 3. 真机运行
python scripts/g1_parkour.py \
    --logdir /path/to/parkour/model \
    --standdir /path/to/stand/model \
    --gimbal --gimbal_port /dev/ttyUSB0 \
    --nodryrun
```

## 必须参数

| 参数 | 说明 |
|------|------|
| `--logdir` | Parkour 策略模型目录 |
| `--standdir` | Stand 策略模型目录 |

## 可选参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--nodryrun` | False | 真机模式（默认 dry-run） |
| `--gimbal` | False | 启用头部云台 |
| `--gimbal_port` | `/dev/ttyUSB0` | 云台串口 |
| `--depth_vis` | False | 发布深度图话题 |
| `--pointcloud_vis` | False | 发布点云话题 |
| `--debug` | False | 远程调试（0.0.0.0:6789） |

## 模型目录结构

```
/path/to/model/
├── params/
│   └── env.yaml           # 从仿真训练导出
└── exported/
    ├── 0-depth_encoder.onnx
    └── actor.onnx
```

## 运行状态机

```
启动 → ColdStart（自动） → 按 R1 → Stand
                                   ↓
                            按 L1 → Parkour
                                   ↓
                            按 R1 → Stand
```

- 任何时刻按 R2/L2：紧急停机
