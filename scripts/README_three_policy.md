# G1 双 Policy 部署说明

当前 `scripts/g1_three_policy.py` 使用两套 policy：

- `29DoF actor-only stand`
- `31DoF parkour`

其中：

- `29DoF stand` 不使用视觉输入
- `29DoF stand` 不控制头部两个自由度
- 头部两个自由度在 `stand` 状态下保持 31DoF 默认位姿
- `31DoF parkour` 继续使用深度相机和头部云台

## 入口脚本

```bash
python scripts/g1_three_policy.py
```

## 启动命令

Dry-run：

```bash
python scripts/g1_three_policy.py \
    --stand_logdir /home/user/hyn/instinct/logs/instinct_rl/g1_parkour/20260505_234657 \
    --logdir /path/to/31dof_parkour \
    --gimbal
```

真机：

```bash
python scripts/g1_three_policy.py \
    --stand_logdir /home/user/hyn/instinct/logs/instinct_rl/g1_parkour/20260505_234657 \
    --logdir /path/to/31dof_parkour \
    --gimbal \
    --nodryrun
```

## 路径说明

- `--stand_logdir`：29DoF 无视觉 stand policy 目录
- `--logdir`：31DoF parkour policy 目录

对当前 `stand` policy，导出目录里只需要：

```text
exported/actor.onnx
```

如果存在 `exported/policy_normalizer.npz`，也会自动加载。

## 状态机

启动后流程如下：

1. 自动进入 `ColdStart`
2. `ColdStart` 完成后，自动进入 `29DoF stand`
3. 在 `29DoF stand` 中按 `L1` 切到 `31DoF parkour`
4. 在 `31DoF parkour` 中按 `R1` 切回 `29DoF stand`

## 按键

`ColdStart` 完成后：

- 自动进入 `29DoF stand`

在 `29DoF stand` 状态：

- `R1`：重新进入 `29DoF stand`
- `L1`：切到 `31DoF parkour`，同时设置 `speed_scale = 0.5`

在 `31DoF parkour` 状态：

- `R1`：切回 `29DoF stand`
- `L1`：保持在 `31DoF parkour`，并设置 `speed_scale = 0.5`

## 相机和头部

当前脚本仍然运行在 `G1_31Dof_TorsoBase` 节点上。

- `29DoF stand`
  - 只控制前 29 个 body joints
  - 头部两个关节固定在默认位姿
  - 不读取深度图

- `31DoF parkour`
  - 控制全部 31 个自由度
  - 使用深度相机
  - 头部两个关节通过 gimbal 控制

## 急停

仍然沿用底层 `UnitreeNode` 逻辑：

- `R2`：急停
- `L2`：急停

## 推荐流程

建议实际使用时：

1. 启动脚本
2. 等 `ColdStart` 完成，确认 `29DoF stand` 稳定
3. 到需要进入感知运动时按 `L1`
4. 需要退回稳定姿态时按 `R1`
