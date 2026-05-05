# G1 Three Policy 部署说明

这个脚本把三个 policy 放进同一个真机部署节点里运行：

- `29DoF stand`
- `29DoF parkour`
- `31DoF parkour`

入口脚本：

```bash
python scripts/g1_three_policy.py
```

## 启动命令

Dry-run：

```bash
python scripts/g1_three_policy.py \
    --stand29_logdir /path/to/stand29 \
    --parkour29_logdir /path/to/parkour29 \
    --parkour31_logdir /path/to/parkour31 \
    --gimbal \
    --camera_serial 420122071680
```

真机：

```bash
cd ~/hmv_deploy
source instinct_venv/bin/activate
source ~/unitree_ros2/setup_g1_foxy.sh
export PYTHONPATH=/home/unitree/ros2_numpy:$PYTHONPATH

python scripts/g1_parkour.py \
  --logdir "/home/unitree/data_model/data&model/checkpoints/parkour_onboard_preview_stair" \
  --standdir "/home/unitree/data_model/data&model/checkpoints/stand_onboard" \
  --depth_vis \
  --pointcloud_vis \
  --nodryrun

python scripts/g1_three_policy.py \
    --stand29_logdir "/home/unitree/data_model/data&model/checkpoints/stand_onboard" \
    --parkour29_logdir "/home/unitree/data_model/data&model/checkpoints/parkour_onboard_preview_stair" \
    --parkour31_logdir "/home/unitree/data_model/data&model/checkpoints/hmv" \
    --gimbal \
    --camera_serial 420122071680 \
    --nodryrun
```

## 路径参数

- `--stand29_logdir`：29DoF stand policy 目录
- `--parkour29_logdir`：29DoF parkour policy 目录
- `--parkour31_logdir`：31DoF parkour policy 目录
- `--camera_serial`：当前使用的 RealSense 序列号
- `--gimbal`：启用头部云台控制
- `--gimbal_port`：头部舵机串口，默认 `/dev/ttyUSB0`

## 按键

状态机会先自动进入 `ColdStart`。

`ColdStart` 完成后：

- `L1`：切到 `29DoF stand`
- `UP`：切到 `29DoF parkour`
- `R1`：切到 `31DoF parkour`

在 `29DoF stand` 状态：

- `UP`：切到 `29DoF parkour`
- `R1`：切到 `31DoF parkour`

在 `29DoF parkour` 状态：

- `L1`：切到 `29DoF stand`
- `R1`：切到 `31DoF parkour`

在 `31DoF parkour` 状态：

- `L1`：切到 `29DoF stand`
- `UP`：切到 `29DoF parkour`

## 推荐切换流程

平地走动时：

- 先进入 `29DoF stand`
- 再切到 `29DoF parkour`

到楼梯或障碍前：

- 先回 `29DoF stand`
- 再切到 `31DoF parkour`

这样切换会比直接在两个运动 policy 之间硬切更稳。

## 当前设备配置

当前代码默认：

- `31DoF parkour` 相机 serial：`420122071680`
- `29DoF perceptive track` 相机 serial：`243622073048`

这个三策略脚本默认使用：

- `--camera_serial 420122071680`

## 说明

- 这个脚本运行在 `G1_31Dof_TorsoBase` 节点上
- `29DoF` policy 通过适配层运行，头部两维固定在默认姿态
- `31DoF parkour` 会正常使用头部两电机和深度相机
- 紧急停机仍然按原系统逻辑处理，`R2/L2` 保留为急停
