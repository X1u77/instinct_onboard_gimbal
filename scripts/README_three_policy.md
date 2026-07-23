# G1 Three-Policy 新头部部署

`scripts/g1_three_policy.py` 同时运行三套策略：

- `stand`：原29-DoF模型，只控制身体关节。
- `walk`：原29-DoF模型，只控制身体关节。
- `parkour`：基于 Humanoid-Monument-Valley `G1Comp` 配置训练的新头部31-DoF模型。

29-DoF策略运行时，policy的观测和动作仍然只有29维。部署层仅为统一31关节
下发接口补出两维固定头部目标：

```text
head_yaw   = 0 rad
head_pitch = 0 rad
```

只有31-DoF parkour policy 会读取和控制头部。其 raw head action 为零时，对应训练
配置的默认位置 `yaw=0、pitch=-0.0926646 rad（约 -5.31°）`。

## 模型目录

```text
stand_logdir/
├── params/env.yaml
└── exported/...

walk_logdir/
├── params/env.yaml
└── exported/actor.onnx

parkour_logdir/
├── params/env.yaml
├── params/agent.yaml
├── exported/actor.onnx
└── exported/0-depth_encoder.onnx
```

Parkour 目录也兼容四个文件直接平铺在同一目录的旧格式。

## 推荐启动流程

部署直接使用仓库中的官方 `g1_comp_servo_service`。先编译并完成三段式标定：

```bash
cd /home/huo/code/instinct_onboard_gimbal/g1_comp_servo_service
sudo apt install libyaml-cpp-dev libspdlog-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/test_calibration --serial /dev/ttyUSB0 --config ./config/config.yaml
```

必须确认生成的 `config.yaml` 中：

```yaml
joint0: [-50, 50]
joint1: [-20, 85]
direction: [1, -1]
has_calibrate: 1
```

然后单独启动官方串口到DDS服务：

```bash
./build/main \
  --network eth0 \
  --domain "${ROS_DOMAIN_ID:-0}" \
  --serial /dev/ttyUSB0 \
  --config ./config/config.yaml
```

`eth0`应替换为机器人DDS使用的网卡，domain必须与three-policy一致。
服务负责DYNAMIXEL SDK、1 Mbps串口、
标定换算、P/D增益和扭矩使能；three-policy只收发DDS命令和状态。

three-policy dry-run只检查模型、观测和相机流程，不会连接或控制头部server：

```bash
python scripts/g1_three_policy.py \
  --stand_logdir /path/to/29dof_stand \
  --walk_logdir /path/to/29dof_walk \
  --logdir /path/to/31dof_d455_parkour \
  --gimbal \
  --debug_policy_io
```

先用下文的官方 `test_read_angle` 核对头部反馈方向，再启动server并运行真机：

```bash
python scripts/g1_three_policy.py \
  --stand_logdir /path/to/29dof_stand \
  --walk_logdir /path/to/29dof_walk \
  --logdir /path/to/31dof_d455_parkour \
  --gimbal \
  --nodryrun
```

未标定或 `has_calibrate != 1` 时，官方服务会拒绝启动。three-policy真机模式会等待
服务的第一帧状态反馈；未启动服务时不会进入控制循环。

D455 默认自动选择第一台设备，默认深度流为 `848×480 @ 60 FPS`。多相机时指定：

```bash
--camera_serial <D455_SERIAL>
```

相机不支持默认 profile 时，可显式设置：

```bash
--camera_width 848 --camera_height 480 --camera_fps 30
```

## 状态机按键

- ColdStart 完成后：
  - `R1`：进入29-DoF stand。
  - `A`：进入29-DoF walk。
  - `L1`：进入31-DoF parkour。
- 任意策略中仍可用上述按键切换。
- `R2` 或 `L2`：急停。

进入 parkour 时默认用0.5秒从当前31关节位置平滑过渡，可用
`--parkour_blend_duration` 调整。`--parkour_freeze_head` 可用于标定，
它会令 parkour 策略也保持新头部默认姿态。

真机模式会等待第一帧云台反馈；运行中反馈超过1秒未更新会立即停止电机命令。
云台连接失败时程序拒绝进入31-DoF部署。parkour运行时D455深度帧超过1秒未更新
也会停止电机命令。

## 新头部坐标约定

新 URDF 的两个轴为：

```text
head_yaw_joint:   axis = (0, 0, -1)，仿真正方向向右
head_pitch_joint: axis = (0, -1, 0)，仿真正方向向上
```

按照文档规定在joint0最右端、joint1最下端执行标定后，官方server源码实际输出：

```text
joint0/yaw:   -50°（右）到 +50°（左）
joint1/pitch: -20°（俯）到 +85°（仰）
```

官方 `utilities.h` 把两个calibration encoder都定义为各自关节下限，因此
joint0服务坐标与新URDF yaw方向相反，joint1与URDF pitch方向一致：

```text
yaw_sign   = -1
pitch_sign = +1
```

调试角度反馈直接使用官方程序：

```bash
./g1_comp_servo_service/build/test_read_angle \
  --serial /dev/ttyUSB0 \
  --config ./g1_comp_servo_service/config/config.yaml
```
