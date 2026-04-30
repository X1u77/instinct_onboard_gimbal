<<<<<<< HEAD
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
=======
# G1 Parkour

在 Unitree G1 机器人上运行 31-DOF 跑酷策略模型，支持 RealSense D435 深度相机感知和头部云台 UART 舵机控制。

仿真中 `head_yaw` 正方向 = 云台左转，UART 舵机正方向 = 云台右转，因此 `joint_signs[29] = -1`。`head_pitch` 方向一致，`joint_signs[30] = +1`。


## 依赖

```bash
pip install -e instinct_onboard_gimbal/
```

确保已安装：`rclpy`, `unitree_hg`, `unitree_go`, `pyrealsense2`, `onnxruntime`。

## 模型准备

>>>>>>> 17f938fb4c26a86fa10d2d00379403cbc700d554

```
/path/to/model/
├── params/
<<<<<<< HEAD
│   └── env.yaml           # 从仿真训练导出
=======
│   └── env.yaml          
>>>>>>> 17f938fb4c26a86fa10d2d00379403cbc700d554
└── exported/
    ├── 0-depth_encoder.onnx
    └── actor.onnx
```

<<<<<<< HEAD
## 运行状态机

```
启动 → ColdStart（自动） → 按 R1 → Stand
                                   ↓
                            按 L1 → Parkour
                                   ↓
                            按 R1 → Stand
```

- 任何时刻按 R2/L2：紧急停机
=======
## 启动方式

### Dry-run 模式

```bash
python scripts/g1_parkour.py \
    --logdir /path/to/parkour/model \
    --standdir /path/to/stand/model \
    --gimbal
```

### 真机运行

```bash
# 终端 1：启动 ROS 2 桥接
source /opt/ros/humble/setup.bash
ros2 launch unitree_bridge unitree_bridge.launch.py

# 终端 2：运行主节点
python scripts/g1_parkour.py \
    --logdir /path/to/parkour/model \
    --standdir /path/to/stand/model \
    --gimbal \
    --gimbal_port /dev/ttyUSB1 \
    --nodryrun
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--logdir` | 必填 | Parkour 模型路径 |
| `--standdir` | 必填 | Stand 模型路径 |
| `--nodryrun` | False | 真机模式（默认 dry-run） |
| `--gimbal` | False | 启用云台舵机控制 |
| `--gimbal_port` | `/dev/ttyUSB0` | 云台串口设备路径 |
| `--startup_step_size` | 0.2 | 启动阶段步长 |
| `--kpkd_factor` | 2.0 | 启动阶段 PD 增益倍数 |
| `--depth_vis` | False | 发布深度图到 `/debug/depth_image` |
| `--pointcloud_vis` | False | 发布点云到 `/debug/pointcloud` |
| `--lin_vel_deadband` | 0.5 | 速度指令死区 |
| `--ang_vel_deadband` | 0.15 | 角速度指令死区 |
| `--debug` | False | 启用 debugpy（监听 `0.0.0.0:6789`） |

## 运行流程

```
上电 / 启动
    ↓
ColdStart
    ↓
按 R1 → Stand Agent
    ↓
按 L1 → Parkour Agent
    ↓
按 R1 → 回到 Stand
```
>>>>>>> 17f938fb4c26a86fa10d2d00379403cbc700d554
