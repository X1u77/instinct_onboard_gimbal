# Instinct Onboard - Unitree G1 真机部署指南

G1-Comp首次上真机请先完整执行
[G1_COMP_REAL_ROBOT_DEPLOYMENT_SOP.md](G1_COMP_REAL_ROBOT_DEPLOYMENT_SOP.md)。

## 目录

- [概述](#概述)
- [硬件准备](#硬件准备)
- [软件环境](#软件环境)
- [网络配置](#网络配置)
- [模型准备](#模型准备)
- [部署脚本一览](#部署脚本一览)
- [Dry-run 测试流程](#dry-run-测试流程)
- [真机部署流程](#真机部署流程)
- [状态机与操控说明](#状态机与操控说明)
- [命令行参数详解](#命令行参数详解)
- [话题与消息](#话题与消息)
- [调试工具](#调试工具)
- [常见问题](#常见问题)

---

## 概述

本仓库包含 Unitree G1 人形机器人在 ROS2 Humble 环境下的完整真机部署代码，支持三种运行模式：

| 模式 | 脚本 | 说明 |
|------|------|------|
| 跑酷（Parkour） | `g1_parkour.py` | 深度相机感知 + 障碍物穿越 |
| 感知追踪（Perceptive Track） | `g1_perceptive_track.py` | 深度相机 + 动作序列播放 |
| 基础追踪（Track） | `g1_track.py` | 无相机 + 动作序列播放 |

**架构概览：**

```
ROS2 (Humble)
├── UnitreeNode          ← 机器人 ROS 接口（关节状态、IMU、遥操作）
├── RsCameraNodeMixin    ← RealSense 相机节点（深度图 IPC）
├── ParkourAgent         ← 跑酷策略（ONNX actor + depth encoder）
├── TrackerAgent         ← 动作追踪策略（ONNX actor）
├── ShadowingAgent       ← 影子跟随策略（ONNX actor + motion ref encoder）
└── WalkAgent            ← 行走策略（ONNX actor）
```

---

## 硬件准备

### 机器人

- **Unitree G1**（支持 29-DOF 和 31-DOF 两种配置）
  - 29-DOF：标准 G1 身体（无头部云台）
  - 31-DOF：G1 + 头部双自由度云台（通过 UART 舵机控制）

### 相机

- Three-policy新头部部署使用 **Intel RealSense D455**
- 通过 USB 3.0 连接到机器人主板或主机
- Three-policy默认深度流：848 x 480 @ 60 FPS
- 确保 IR 投射器正常工作

### G1-Comp头部

- 双轴DYNAMIXEL头部，Protocol 2.0
- 必须先运行官方 `g1_comp_servo_service/test_calibration`
- 官方server独占串口并通过DDS向three-policy提供控制和反馈
- 默认串口：`/dev/ttyUSB0`，官方server使用1 Mbps

### 工控机要求

- NVIDIA GPU
- Ubuntu 22.04 + ROS2 Humble

---

## 软件环境

### 1. ROS2 Humble

参考官方文档安装：https://docs.ros.org/en/humble/Installation.html

```bash
sudo apt update && sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 2. 依赖安装

```bash
# 克隆代码
cd ~/catkin_ws/src
git clone <repo_url> instinct_onboard

# 安装 Python 依赖
cd instinct_onboard/instinct_onboard_gimbal
pip install -e .  # 会自动检测 GPU 并选择 onnxruntime-gpu 或 onnxruntime

# 如果手动安装：
pip install numpy numpy-quaternion pyyaml opencv-python
pip install onnxruntime-gpu  # GPU 版本
# 或
pip install onnxruntime      # CPU 版本
```

### 3. ROS2 功能包

```bash
# Unitree 机器人接口
sudo apt install ros-humble-unitree-go ros-humble-unitree-hg

# Realsense SDK
sudo apt install librealsense2-dev ros-humble-realsense2-camera
# 或从源码编译 pyrealsense2
pip install pyrealsense2

# 其他 ROS 依赖
pip install ros2_numpy
```

### 4. 串口权限

```bash
# 将当前用户添加到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录后生效

# 或直接设置权限
sudo chmod 666 /dev/ttyUSB0
```

### 5. 验证 RealSense

```bash
# 查看相机
rs-enumerate-devices

# 测试 ROS 发布
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x60
```

### 6. 验证 Unitree 消息

```bash
# 查看 Unitree LowState 话题
ros2 topic list | grep low
ros2 topic echo /lowstate  # 应该有数据输出
```

---

## 网络配置

### 工控机与机器人连接

确保工控机与机器人控制箱在同一网络：

```bash
# 假设机器人控制箱 IP 为 192.168.123.12
# 工控机 IP 设置为同一网段，如 192.168.123.10
sudo ip addr add 192.168.123.10/24 dev eth0
```

### ROS2 Domain

建议设置独立的 Domain 避免干扰：

```bash
export ROS_DOMAIN_ID=42
```

---

## 模型准备

### 目录结构

每个策略模型目录需要包含以下文件：

```
/path/to/model/
├── params/
│   ├── env.yaml          # 仿真环境配置（从 IsaacLab 导出）
│   └── agent.yaml        # Agent 专用配置
└── exported/
    ├── actor.onnx                    # 主策略网络
    ├── policy_normalizer.npz         # 观测归一化参数（均值、标准差）
    ├── 0-depth_encoder.onnx         # 深度编码器（仅感知策略）
    ├── 0-motion_ref.onnx            # 动作参考编码器（仅影子跟随）
    └── forward_kinematics.onnx      # 前向运动学（仅影子跟随）
```

### 动作序列文件（追踪模式）

动作序列文件为 `.npz` 格式，通过 retargeting 生成：

```
/path/to/motions/
├── diveroll4-ziwen-0-retargeted.npz
├── kneelClimbStep1-x-0.1-ziwen-retargeted.npz
├── rollVault11-ziwen-retargeted.npz
├── jumpsit2-ziwen-retargeted.npz
└── superheroLanding-retargeted.npz
```

---

## 部署脚本一览

### g1_parkour.py

**功能：** 跑酷模式，支持深度相机感知和头部云台控制。

```bash
python scripts/g1_parkour.py \
    --logdir /path/to/parkour/model \
    --standdir /path/to/stand/model
```

### g1_perceptive_track.py

**功能：** 感知追踪模式，深度相机 + 动作序列播放。

```bash
python scripts/g1_perceptive_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions \
    --walk_logdir /path/to/walk/model
```

### g1_track.py

**功能：** 基础追踪模式，无相机 + 动作序列播放。

```bash
python scripts/g1_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions
```

### g1_shadowing.py

**功能：** 影子跟随模式，通过 ROS 话题实时接收动作参考。

```bash
python scripts/g1_shadowing.py \
    --logdir /path/to/shadowing/model
```

### rs_cam_test.py

**功能：** RealSense 相机测试，发布深度图和点云话题。

```bash
python scripts/rs_cam_test.py
```

---

## Dry-run 测试流程

**强烈建议在连接真机前先进行 Dry-run 测试。**

### 步骤 1：启动 ROS

```bash
# 终端 1：启动 ROS
ros2 run demo_nodes_cpp listener &
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
```

### 步骤 2：启动 Unitree 模拟（如果有）

如果没有真实机器人，可以启动 Unitree 的模拟节点：

```bash
# 终端 2：启动 Unitree LowState 模拟
ros2 topic pub /lowstate unitree_hg/msg/LowState "{...}" -r 50
```

### 步骤 3：运行脚本

```bash
# Dry-run 模式（默认），不会发送真机命令
cd instinct_onboard_gimbal

# 测试跑酷模式
python scripts/g1_parkour.py \
    --logdir /path/to/parkour/model \
    --standdir /path/to/stand/model \
    --depth_vis \
    --pointcloud_vis

# 测试追踪模式
python scripts/g1_perceptive_track.py \
    --logdir /path/to/tracking/model \
    --motion_dir /path/to/motions \
    --motion_vis
```

### 步骤 4：观察输出

- 终端应显示 `Actual main loop frequency: XX Hz`（目标 50 Hz）
- 深度图话题 `/debug/depth_image` 应有数据
- 点云话题 `/debug/pointcloud` 应有数据
- 关节状态话题 `/raw_actions` 应发布动作数据

---

## 真机部署流程

### 步骤 1：安全检查

- [ ] 机器人周围无障碍物，足够运动空间
- [ ] 操作人员站在安全距离外
- [ ] 急停按钮触手可及
- [ ] 所有线缆连接牢固

### 步骤 2：启动 ROS

```bash
# 终端 1
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 daemon start
```

### 步骤 3：启动 Unitree 节点

```bash
# 终端 2
ros2 run unitree_hg unitree_hg_ros
# 或启动你自己的 Unitree 桥接节点
```

### 步骤 4：启动 RealSense

```bash
# 终端 3
ros2 launch realsense2_camera rs_launch.py \
    depth_module.profile:=480x270x60 \
    rgb_camera.profile:=424x240x60 \
    enable_depth:=true
```

### 步骤 5：运行策略脚本

```bash
# 终端 4
cd instinct_onboard_gimbal
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

# 跑酷模式（推荐首次运行）
python scripts/g1_parkour.py \
    --logdir /path/to/parkour/model \
    --standdir /path/to/stand/model \
    --gimbal \
    --gimbal_port /dev/ttyUSB0 \
    --depth_vis \
    --pointcloud_vis \
    --nodryrun
```

### 步骤 6：观察并准备接管

- 脚本启动后会进入 **ColdStart** 阶段，机器人自动移动到初始姿态
- 终端显示 `ColdStartAgent done` 后，按 R1 进入 Stand 模式
- 按 L1 进入 Parkour 模式，开始接受遥操作命令
- 任何时刻按 R2/L2 触发紧急停机

---

## 状态机与操控说明

### 状态机

```
上电/启动
    ↓
ColdStart（自动）  ←  机器人自动移动到初始姿态
    ↓ (完成后)
Stand（按 R1）  ←  站立平衡模式
    ↓ (按 L1)
Parkour  ←  跑酷/导航模式（接受遥操作）
    ↓ (按 R1)
Stand
```

### 手柄按键映射（Unitree 无线手柄）

| 按键 | 功能 |
|------|------|
| **R1** | 进入/返回 Stand 站立模式 |
| **L1** | 进入 Parkour 跑酷模式 |
| **R2** | 紧急停机（按住生效） |
| **L2** | 紧急停机（按住生效） |
| 左遥杆 | 前进/后退、左移/右移 |
| 右遥杆 | 转向 |
| 方向键 ↑ | 加载动作序列 1（追踪模式） |
| 方向键 ↓ | 加载动作序列 2（追踪模式） |
| 方向键 ← | 加载动作序列 3（追踪模式） |
| 方向键 → | 加载动作序列 4（追踪模式） |
| **X** | 加载动作序列 5（追踪模式） |
| **A** | 匹配当前航向（追踪模式） |

### 速度参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--lin_vel_deadband` | 0.5 | 线性速度遥杆死区 |
| `--lin_vel_range` | [0.5, 0.5] | 线性速度范围（前进方向） |
| `--ang_vel_deadband` | 0.15 | 角速度遥杆死区 |
| `--ang_vel_range` | [0.0, 1.0] | 角速度范围 |

---

## 命令行参数详解

### 通用参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--nodryrun` | flag | False | 启用真机模式（默认 Dry-run） |
| `--debug` | flag | False | 启用 debugpy 远程调试（0.0.0.0:6789） |
| `--startup_step_size` | float | 0.2 | ColdStart 关节步进大小 |
| `--kpkd_factor` | float | 2.0 | ColdStart 的 KP/KD 增益倍数 |

### 相机参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--depth_vis` | flag | False | 发布深度图话题 `/debug/depth_image` |
| `--pointcloud_vis` | flag | False | 发布点云话题 `/debug/pointcloud` |
| `--motion_vis` | flag | False | 发布动作序列关节状态（用于 RViz 可视化） |

### 云台参数（旧 `g1_parkour.py` 入口）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--gimbal` | flag | False | 启用头部云台控制 |
| `--gimbal_port` | string | `/dev/ttyUSB0` | 云台串口设备路径 |
| `--gimbal_pan_range` | float | `[-90, 90]` | 云台水平角度范围（度） |
| `--gimbal_tilt_range` | float | `[-45, 45]` | 云台俯仰角度范围（度） |

当前 `g1_three_policy.py` 使用官方G1-Comp DDS server，不接受串口、波特率、
ID或手工range参数。串口和标定文件在启动server时指定。

### 速度控制参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--lin_vel_deadband` | float | 0.5 | 线性速度死区（0-1） |
| `--lin_vel_range` | 2 floats | [0.5, 0.5] | 前进/后退速度范围 |
| `--ang_vel_deadband` | float | 0.15 | 角速度死区（0-1） |
| `--ang_vel_range` | 2 floats | [0.0, 1.0] | 左右转向速度范围 |

---

## 话题与消息

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/raw_actions` | `std_msgs/Float32MultiArray` | 原始策略输出动作 |
| `/debug/depth_image` | `sensor_msgs/Image` | 归一化深度图（调试用） |
| `/debug/pointcloud` | `sensor_msgs/PointCloud2` | 深度点云（调试用） |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态（用于 RViz） |
| `/gimbal/joint_states` | `sensor_msgs/JointState` | 云台关节状态 |
| `/gimbal/current_angle` | `std_msgs/Float32MultiArray` | 云台当前角度 |
| `/gimbal/health` | `std_msgs/Bool` | 云台健康状态 |

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/lowstate` | `unitree_hg/msg/LowState` | Unitree 机器人状态 |
| `/secondary_imu` | `unitree_hg/msg/IMUState` | 机器人 IMU 数据 |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | 手柄遥操作数据 |
| `/gimbal/target_angle` | `std_msgs/Float32MultiArray` | 云台目标角度 |
| `/motion_sequence` | `motion_target_msgs/msg/MotionSequence` | 动作序列（影子跟随） |

### TF 变换

| 父 frame | 子 frame | 说明 |
|----------|----------|------|
| `torso_link` | `realsense_depth_link` | RealSense 相机位置 |
| `base_link` | `pan_link` | 云台水平轴 |
| `pan_link` | `tilt_link` | 云台俯仰轴 |

---

## 调试工具

### 1. 远程调试（debugpy）

```bash
# 添加 --debug 参数，监听 0.0.0.0:6789
python scripts/g1_parkour.py --logdir /path --debug
```

在 VS Code 中配置 `launch.json`：

```json
{
    "name": "Python: Remote Attach",
    "type": "python",
    "request": "attach",
    "host": "<机器人IP>",
    "port": 6789
}
```

### 2. 深度图可视化

```bash
# 启用深度图话题发布
python scripts/g1_parkour.py --logdir /path --depth_vis

# 在另一终端用 rqt_image_view 查看
rqt_image_view /debug/depth_image
```

### 3. 点云可视化

```bash
# 启用点云话题发布
python scripts/g1_parkour.py --logdir /path --pointcloud_vis

# 在 RViz 中添加 PointCloud2 显示
rviz2
```

### 4. 关节状态可视化

```bash
# 发布动作序列的关节状态
python scripts/g1_perceptive_track.py --logdir /path --motion_dir /motions --motion_vis

# 在 RViz 中加载 G1 URDF 模型
rviz2
# 添加 RobotModel 和 TF 显示
```

### 5. 频率监控

终端会定期打印实际运行频率：

```
Actual main loop frequency: 49.85 Hz. Mean time consumption: 0.0182 s.
```

- 目标频率：50 Hz（周期 20ms）
- 如果频率明显低于 50 Hz，检查 ONNX 推理时间和 ROS 回调延迟

### 6. 舵机调试

```bash
# 扫描舵机
python instinct_onboard/test_servo.py --scan

# 测试连接
python instinct_onboard/test_servo.py --port /dev/ttyUSB0

# 测试角度控制
python instinct_onboard/test_servo.py --port /dev/ttyUSB0 --test-angle 30 15

# 云台扫描测试
python instinct_onboard/test_servo.py --port /dev/ttyUSB0 --sweep

# 模拟模式（不连接硬件）
python instinct_onboard/test_servo.py --dryrun
```

---

## 常见问题

### Q1: `ImportError: No module named 'unitree_go'`

Unitree 消息包未安装或未 source：

```bash
sudo apt install ros-humble-unitree-go ros-humble-unitree-hg
source /opt/ros/humble/setup.bash
```

### Q2: `RuntimeError: Camera process is not alive`

RealSense 相机进程意外退出，检查：

- USB 3.0 连接是否稳定
- 是否有其他进程占用相机
- 相机固件是否为最新

```bash
# 重置 USB
echo -1 | sudo tee /sys/bus/usb/drivers/usb/unbind
echo 1-2 | sudo tee /sys/bus/usb/drivers/usb/bind
```

### Q3: 关节位置超出保护范围

```
Joint 5(sim), 13(real) position out of range at 2.53
The motors and this process shuts down.
```

机器人关节位置异常，自动停机。可能原因：

- 初始姿态不对
- 电机零点漂移
- 通信延迟导致动作积分误差

### Q4: 策略动作与预期不符

- 检查模型是否与仿真训练配置一致
- 确认 `env.yaml` 从仿真正确导出
- 确认 `policy_normalizer.npz` 包含正确的归一化参数

### Q5: 深度图全黑或全白

- 归一化参数与实际深度范围不匹配
- 检查 `env.yaml` 中的 `depth_range` 设置
- 确认 RealSense 深度尺度正确（`depth_scale`）

### Q6: 云台舵机无响应

```bash
# 检查串口权限
ls -la /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0

# 使用官方程序读取G1-Comp标定后的关节角
g1_comp_servo_service/build/test_read_angle \
  --serial /dev/ttyUSB0 \
  --config g1_comp_servo_service/config/config.yaml
```

### Q7: `onnxruntime` GPU 推理报错

```bash
# 检查 GPU 可用性
python -c "import onnxruntime as ort; print(ort.get_available_providers())"

# 强制使用 CPU
export FORCE_CPU=1
python scripts/g1_parkour.py ...
```

### Q8: ROS2 话题无数据

```bash
# 检查话题列表
ros2 topic list

# 查看话题带宽
ros2 topic bw /lowstate

# 查看消息频率
ros2 topic hz /lowstate
```

---

## 附录：完整命令示例

### Three-policy模式（29-DoF站立/行走 + 31-DoF跑酷）

终端1：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
g1_comp_servo_service/build/main \
    --network eth0 \
    --domain "$ROS_DOMAIN_ID" \
    --serial /dev/ttyUSB0 \
    --config g1_comp_servo_service/config/config.yaml
```

终端2：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export CUDA_VISIBLE_DEVICES=0
python scripts/g1_three_policy.py \
    --stand_logdir /models/stand_g1_29dof \
    --walk_logdir /models/walk_g1_29dof \
    --logdir /models/parkour_g1_comp_31dof \
    --gimbal \
    --startup_step_size 0.2 \
    --kpkd_factor 2.0 \
    --depth_vis \
    --pointcloud_vis \
    --nodryrun
```

### 感知追踪模式（真机 + 动作序列）

```bash
python scripts/g1_perceptive_track.py \
    --logdir /models/perceptive_track_g1 \
    --motion_dir /motions/retargeted \
    --walk_logdir /models/walk_g1 \
    --motion_vis \
    --depth_vis \
    --pointcloud_vis \
    --startup_step_size 0.3 \
    --kpkd_factor 1.5 \
    --nodryrun
```
