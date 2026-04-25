# G1 Parkour

在 Unitree G1 机器人上运行 31-DOF 跑酷策略模型，支持 RealSense D435 深度相机感知和头部云台 UART 舵机控制。

仿真中 `head_yaw` 正方向 = 云台左转，UART 舵机正方向 = 云台右转，因此 `joint_signs[29] = -1`。`head_pitch` 方向一致，`joint_signs[30] = +1`。


## 依赖

```bash
pip install -e instinct_onboard_gimbal/
```

确保已安装：`rclpy`, `unitree_hg`, `unitree_go`, `pyrealsense2`, `onnxruntime`。

## 模型准备


```
/path/to/model/
├── params/
│   └── env.yaml          
└── exported/
    ├── 0-depth_encoder.onnx
    └── actor.onnx
```

## 启动方式

### Dry-run 模式（不连接真机、不连舵机）

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
ColdStart（自动）→ 机器人移动到初始姿态
    ↓
按 R1 → Stand Agent（站立平衡）
    ↓
按 L1 → Parkour Agent（深度感知自主跑酷）
    ↓
按 R1 → 回到 Stand
```

## 安全机制

- **R2 / L2 按住**：紧急停机（所有电机断电 + 云台释放）
- **关节限位保护**：`joint_pos_protect_ratio=2.0`，超限自动停机
- **NaN 检查**：action 含 NaN 时跳过发送
- **扭矩限幅**：`clip_by_torque_limit` 防止输出扭矩过大
- **云台限位**：pan ±90°、tilt ±45°

## 坐标系转换

**读取**：`sim_pos = real_q * joint_signs`

**写入**：`real_q = target_pos * joint_signs`

**云台**：
- 读取：`(servo_deg * 0.1) * joint_signs` → 弧度
- 写入：`sim_rad * joint_signs` → 舵机度
