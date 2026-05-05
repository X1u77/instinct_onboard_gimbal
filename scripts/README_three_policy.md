# G1 单 Policy 部署说明

当前 `scripts/g1_three_policy.py` 已经改成单一 `31DoF parkour policy` 部署脚本。

不再包含：

- `29DoF stand`
- `29DoF parkour`
- 多 policy 切换逻辑

现在只有一个 `31DoF parkour` policy，通过两个按键切换两个固定速度档位。

## 入口脚本

```bash
python scripts/g1_three_policy.py
```

## 启动命令

Dry-run：

```bash
cd ~/hmv_deploy
source hmv_venv/bin/activate
source ~/unitree_ros2/setup_g1_foxy.sh
export PYTHONPATH=/home/unitree/ros2_numpy:$PYTHONPATH

python scripts/g1_three_policy.py \
    --logdir /home/unitree/hmv_deploy/policy \
    --gimbal
```

真机：

```bash
cd ~/hmv_deploy
source hmv_venv/bin/activate
source ~/unitree_ros2/setup_g1_foxy.sh
export PYTHONPATH=/home/unitree/ros2_numpy:$PYTHONPATH

python scripts/g1_three_policy.py \
    --logdir /home/unitree/hmv_deploy/policy \
    --gimbal \
    --nodryrun
```

如果云台串口不是默认值 `/dev/ttyUSB0`，再补：

```bash
--gimbal_port /dev/ttyUSB1
```

## 路径说明

- `--logdir`：31DoF parkour policy 目录
- 当前代码兼容两种目录结构：
  - 标准结构：`params/env.yaml`、`params/agent.yaml`、`exported/actor.onnx`、`exported/0-depth_encoder.onnx`
  - 平铺结构：`env.yaml`、`agent.yaml`、`actor.onnx`、`0-depth_encoder.onnx`

你现在这套 policy 可以直接传：

```bash
--logdir /home/unitree/hmv_deploy/policy
```

## 相机

这个脚本现在默认固定使用唯一的 RealSense：

```text
420122071680
```

所以启动命令里不再需要传 `--camera_serial`。

## 状态机

脚本启动后流程如下：

1. 自动进入 `ColdStart`
2. `ColdStart` 完成后，自动进入唯一的 `31DoF parkour policy`
3. 之后通过按键切换速度档位

## 按键

在 `ColdStart` 完成并进入 policy 后：

- `R1`：设置 `speed_scale = 0.0`
- `L1`：设置 `speed_scale = 0.5`

含义：

- `speed_scale = 0.0`：站立/不前进
- `speed_scale = 0.5`：以训练时最大前进速度的一半运行

这套 policy 的训练配置里：

- `max_velocity = 1.5 m/s`

因此：

- `scale = 0.0` -> `target_vx = 0.0 m/s`
- `scale = 0.5` -> `target_vx = 0.75 m/s`

## 急停

仍然沿用底层 `UnitreeNode` 的急停逻辑：

- `R2`：急停
- `L2`：急停

任一按下都会停电机并退出进程。

## 当前实现说明

- 机器人配置：`G1_31Dof_TorsoBase`
- 头部两自由度由云台控制
- `velocity_commands` 已按训练配置改成 1 维 `speed_scale`
- 不再向 policy 输入 3 维遥控速度

## 推荐使用方式

建议流程：

1. 上电后先启动脚本
2. 等 `ColdStart` 完成
3. 先按 `R1`，确认 `scale=0.0` 时站稳
4. 再按 `L1`，切到 `scale=0.5`

## 当前命令模板

```bash
cd ~/hmv_deploy
source hmv_venv/bin/activate
source ~/unitree_ros2/setup_g1_foxy.sh
export PYTHONPATH=/home/unitree/ros2_numpy:$PYTHONPATH

python scripts/g1_three_policy.py \
    --logdir /home/unitree/hmv_deploy/policy \
    --gimbal \
    --nodryrun
```
