# G1-Comp 真机测试与 Three-Policy 部署 SOP

适用代码：

```text
/home/huo/code/instinct_onboard_gimbal
```

适用策略组合：

- stand：29-DoF，头部固定 `yaw=0、pitch=0`
- walk：29-DoF，头部固定 `yaw=0、pitch=0`
- parkour：31-DoF，使用G1-Comp头部和D455

本文按“第一次在真机上运行”的安全顺序编写。不要跳过标定、方向检查和支撑测试。

---

## 0. 开始前准备

### 0.1 人员和机械安全

- 至少两人在场：一人操作电脑，一人看护机器人和急停。
- 第一次ColdStart、头部控制和策略切换时，机器人必须处于可靠支撑或吊装状态。
- 清空机器人周围空间，确认头部、D455和线缆在全行程内不会碰撞。
- 遥控器保持在线，确认 `R2`、`L2`可用。
- 物理急停应随时可触达；软件急停不能替代物理急停。
- 所有测试开始时摇杆保持中位。

### 0.2 禁止事项

- 不要同时运行两个会访问头部串口的程序。
- 运行 `test_calibration`、其他舵机测试或DYNAMIXEL Wizard时，不要运行server。
- 运行server后，不要再启动任何直接访问同一串口的测试程序。
- 不要复制另一台头部的 `servo0_calibration` 或 `servo1_calibration`。
- 头部反馈方向或零位不正确时，不要启动 `--nodryrun`。
- 模型维度检查失败时，不要通过删除检查或截断数组强行运行。

---

## 1. 设置本次部署参数

进入仓库：

```bash
cd /home/huo/code/instinct_onboard_gimbal
```

根据真机设置以下变量。变量只在当前终端有效：

```bash
export G1_DEPLOY_REPO=/home/huo/code/instinct_onboard_gimbal
export G1_HEAD_PORT=/dev/ttyUSB0
export G1_DDS_IFACE=eth0
export ROS_DOMAIN_ID=42
export G1_UNITREE_SETUP="REPLACE_WITH_UNITREE_ROS2_INSTALL_SETUP"

export G1_STAND_MODEL=/absolute/path/to/29dof_stand
export G1_WALK_MODEL=/absolute/path/to/29dof_walk
export G1_PARKOUR_MODEL=/absolute/path/to/31dof_d455_parkour
```

说明：

- 如果真机上的仓库不在 `/home/huo/code`，先修改 `G1_DEPLOY_REPO`。
- `G1_DDS_IFACE`必须换成机器人DDS实际使用的网卡。
- 如果Unitree消息不是系统安装的，把 `G1_UNITREE_SETUP`换成其
  `install/setup.bash`绝对路径；如果消息已经全局可用，可不source该变量。
- 三个模型路径必须使用绝对路径。
- 所有运行server和three-policy的终端必须设置相同的 `ROS_DOMAIN_ID`。
- 每打开一个新终端，都要重新执行本节的变量设置；不要假设变量会自动继承。

查看网卡：

```bash
ip -br link
ip -br addr
```

查看USB串口：

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/ 2>/dev/null
```

如果存在 `/dev/serial/by-id/...`，优先把 `G1_HEAD_PORT` 设置成该稳定路径。

确认没有其他程序占用串口：

```bash
fuser -v "$G1_HEAD_PORT"
```

正常情况下此时不应有输出。

---

## 2. 检查软件、模型和硬件

### 2.1 加载ROS2 Foxy和Unitree消息环境

```bash
source /opt/ros/foxy/setup.bash
```

如果Unitree消息是从工作空间编译的，还需要加载对应工作空间，例如：

```bash
find /home -type f -path "*/install/setup.bash" 2>/dev/null
source "$G1_UNITREE_SETUP"
```

确认Python消息存在：

```bash
python -c "from unitree_go.msg import MotorCmds, MotorStates; from unitree_hg.msg import LowCmd, LowState; print('Unitree ROS messages: OK')"
```

### 2.2 检查模型文件

```bash
test -f "$G1_STAND_MODEL/params/env.yaml"
test -f "$G1_WALK_MODEL/params/env.yaml"
test -f "$G1_WALK_MODEL/exported/actor.onnx"
test -f "$G1_PARKOUR_MODEL/params/env.yaml"
test -f "$G1_PARKOUR_MODEL/params/agent.yaml"
test -f "$G1_PARKOUR_MODEL/exported/actor.onnx"
test -f "$G1_PARKOUR_MODEL/exported/0-depth_encoder.onnx"
```

以上命令全部没有报错才继续。如果parkour模型采用旧的平铺格式，确认目录中至少有：

```text
env.yaml
agent.yaml
actor.onnx
0-depth_encoder.onnx
```

不要混用不同训练run导出的 `env.yaml`、`agent.yaml` 和ONNX文件。

### 2.3 检查D455

```bash
rs-enumerate-devices | rg "Name|Serial Number|Product Line"
```

记录D455序列号：

```bash
export G1_D455_SERIAL="REPLACE_WITH_D455_SERIAL"
```

确认设备是D455而不是主机上的其他RealSense。

### 2.4 检查G1基础DDS

```bash
ros2 topic hz /lowstate
```

应持续收到接近机器人正常发布频率的数据。然后检查遥控器：

```bash
ros2 topic echo --once /wirelesscontroller
```

如果 `/lowstate` 或 `/wirelesscontroller` 没有数据，先解决Unitree网络、DDS配置或
`ROS_DOMAIN_ID`问题，不要继续真机部署。

---

## 3. 编译官方G1-Comp server

进入server目录：

```bash
cd "$G1_DEPLOY_REPO/g1_comp_servo_service"
```

安装构建依赖：

```bash
sudo apt update
sudo apt install libyaml-cpp-dev libspdlog-dev cmake build-essential
```

Unitree SDK2及CycloneDDS开发库也必须已经安装。检查：

```bash
test -d /usr/local/include/unitree
ldconfig -p | rg "unitree_sdk2|ddsc"
```

编译：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

确认程序生成：

```bash
test -x build/main
test -x build/test_calibration
test -x build/test_read_encoder
test -x build/test_read_angle
./build/main --help
```

如果缺少 `unitree/...`、`ddscxx/...` 头文件或 `unitree_sdk2` 库，需要先按
Unitree SDK2安装说明补齐环境，不能跳过server编译。

### 3.1 串口权限

推荐把当前用户加入 `dialout`：

```bash
sudo usermod -aG dialout "$USER"
```

该设置通常需要注销并重新登录。重新登录后检查：

```bash
groups
test -r "$G1_HEAD_PORT"
test -w "$G1_HEAD_PORT"
```

---

## 4. 头部被动通信测试

确保：

- server没有运行；
- 其他舵机测试没有运行；
- 头部处于可手动缓慢转动的状态；
- 机器人身体仍被可靠支撑。

运行被动encoder读取：

```bash
cd "$G1_DEPLOY_REPO/g1_comp_servo_service"
./build/test_read_encoder --serial "$G1_HEAD_PORT"
```

此版本的读取测试会先明确关闭两个舵机的扭矩，不会使能头部。缓慢手动移动两个关节，
确认：

- `servo 0 position`会随yaw移动而变化；
- `servo 1 position`会随pitch移动而变化；
- 数值不是长期固定、乱码或通信错误。

按 `Ctrl+C`结束。

出现以下情况必须停止：

- 任一舵机没有数据；
- 只有一个ID有数据；
- 读取过程中头部自行运动；
- 串口大量报错或程序无法稳定运行。

---

## 5. 执行三段式标定

### 5.1 备份配置

```bash
cd "$G1_DEPLOY_REPO/g1_comp_servo_service"
cp -i config/config.yaml config/config.yaml.before_calibration
sed -n '1,20p' config/config.yaml
```

标定前配置应包含：

```yaml
joint0: [-50, 50]
joint1: [-20, 85]
direction: [1, -1]
```

### 5.2 标定动作

运行：

```bash
./build/test_calibration \
  --serial "$G1_HEAD_PORT" \
  --config "$G1_DEPLOY_REPO/g1_comp_servo_service/config/config.yaml"
```

程序有三段倒计时：

1. 第一段5秒：
   - 缓慢把 `joint0/yaw` 手动摆到机械允许的最右侧。
   - 保持在最右侧，直到程序打印 `servo 0 position`。

2. 第二段2秒：
   - 手离开joint0和运动范围。
   - 倒计时结束后，joint0会自动回到0°附近。

3. 第三段5秒：
   - 缓慢把 `joint1/pitch` 向下压到机械允许的最低位置。
   - 保持到程序打印 `servo 1 position`。

程序完成后应打印：

```text
Calibration complete. Both head servos are released.
```

如果摆错方向、碰撞、没有到达极限或者过程中断电，必须重新完整标定。
如果标定程序被 `Ctrl+C` 或异常打断，应先断开头部电源，或启动一次
`test_read_encoder`明确释放扭矩，再重新完整标定。

### 5.3 检查标定文件

```bash
sed -n '1,20p' config/config.yaml
```

必须看到：

```yaml
servo0_calibration: <本机实际encoder值>
servo1_calibration: <本机实际encoder值>
joint0: [-50, 50]
joint1: [-20, 85]
direction: [1, -1]
has_calibrate: 1
```

不要手工修改两个calibration值。

---

## 6. 标定后被动角度和方向检查

运行：

```bash
./build/test_read_angle \
  --serial "$G1_HEAD_PORT" \
  --config "$G1_DEPLOY_REPO/g1_comp_servo_service/config/config.yaml"
```

此读取测试会先关闭两个舵机扭矩。手动缓慢移动头部并核对server坐标：

| 物理姿态 | 官方server预期反馈 |
|---|---:|
| yaw最右 | joint0约 `-50°` |
| yaw居中 | joint0约 `0°` |
| yaw最左 | joint0约 `+50°` |
| pitch最低/向下 | joint1约 `-20°` |
| pitch约水平零位 | joint1约 `0°` |
| pitch向上 | joint1为正，最高约 `+85°` |

部署层会对yaw乘 `-1`，因此模型/URDF坐标中仍然是“向右为正”。pitch不取反。

按 `Ctrl+C`结束。

以下任一情况出现时不要deploy：

- 中位明显不是0°附近；
- yaw最右不是约 `-50°`；
- pitch向下不是负值；
- 反馈跳变、卡在限位或明显超出配置范围；
- 相同姿态每次读数差异很大。

这时应检查机械安装和标定动作，重新执行第5节，不要先改代码里的sign。

---

## 7. 启动官方server并检查DDS

先在终端A、终端B分别重新执行第1节的变量设置。

### 7.1 终端A：启动server

```bash
cd "$G1_DEPLOY_REPO/g1_comp_servo_service"
source /opt/ros/foxy/setup.bash
source "$G1_UNITREE_SETUP"
export ROS_DOMAIN_ID=42

./build/main \
  --network "$G1_DDS_IFACE" \
  --domain "$ROS_DOMAIN_ID" \
  --serial "$G1_HEAD_PORT" \
  --config "$G1_DEPLOY_REPO/g1_comp_servo_service/config/config.yaml"
```

把示例中的Unitree工作空间路径换成真机实际路径。如果消息已安装到系统，可省略第二个
`source`。

server启动后会设置P/D增益并持续发布状态，但在收到有效 `mode=1` 命令前不会使能扭矩。

### 7.2 终端B：检查DDS

```bash
source /opt/ros/foxy/setup.bash
source "$G1_UNITREE_SETUP"
export ROS_DOMAIN_ID=42

ros2 topic list | rg "g1_comp_servo"
ros2 topic hz /g1_comp_servo/state
ros2 topic echo --once /g1_comp_servo/state
```

必须看到：

```text
/g1_comp_servo/cmd
/g1_comp_servo/state
```

`state`应持续发布两个motor state，`q`为角度制。

如果看不到话题：

1. 确认server与ROS2终端的 `ROS_DOMAIN_ID`一致；
2. 确认 `--network`是正确网卡；
3. 确认两个终端加载了相同的CycloneDDS/Unitree环境；
4. 确认server没有串口或标定错误。

---

## 8. Three-policy dry-run

Dry-run用于检查模型、观测、D455和状态机，不发送真实身体或头部控制命令。

在终端B运行：

```bash
cd "$G1_DEPLOY_REPO"
source /opt/ros/foxy/setup.bash
source "$G1_UNITREE_SETUP"
export ROS_DOMAIN_ID=42

python scripts/g1_three_policy.py \
  --stand_logdir "$G1_STAND_MODEL" \
  --walk_logdir "$G1_WALK_MODEL" \
  --logdir "$G1_PARKOUR_MODEL" \
  --camera_serial "$G1_D455_SERIAL" \
  --gimbal \
  --parkour_freeze_head \
  --debug_policy_io \
  --depth_vis
```

检查：

- 三个模型全部成功加载；
- 没有actor input/output维度错误；
- parkour通过新G1Comp/D455模型配对检查；
- D455持续产生有效深度帧；
- 主循环接近50 Hz；
- dry-run lowcmd发布到带随机后缀的安全话题，而不是 `/lowcmd`；
- 遥控器按键能够切换状态，但机器人不运动。

可以检查：

```bash
ros2 topic list | rg "lowcmd_dryrun|debug/depth_image|raw_actions"
ros2 topic hz /debug/depth_image
```

按 `Ctrl+C`结束，确认打印：

```text
Node shutdown complete.
```

模型加载、深度图或维度检查有任何错误都应先修复，不能进入下一阶段。

---

## 9. 第一次真机控制：支撑状态、冻结parkour头部

这是第一次会发送真实 `/lowcmd` 和头部命令的步骤。

确认：

- 机器人可靠吊装或支撑；
- server仍在终端A正常运行；
- `/g1_comp_servo/state`持续更新；
- D455正常；
- 所有摇杆处于中位；
- 看护人员握住物理急停；
- 操作人员随时准备按 `R2` 或 `L2`。

使用保守ColdStart参数，并冻结parkour头部：

```bash
cd "$G1_DEPLOY_REPO"
source /opt/ros/foxy/setup.bash
source "$G1_UNITREE_SETUP"
export ROS_DOMAIN_ID=42

python scripts/g1_three_policy.py \
  --stand_logdir "$G1_STAND_MODEL" \
  --walk_logdir "$G1_WALK_MODEL" \
  --logdir "$G1_PARKOUR_MODEL" \
  --camera_serial "$G1_D455_SERIAL" \
  --gimbal \
  --parkour_freeze_head \
  --parkour_blend_duration 1.0 \
  --startup_step_size 0.02 \
  --kpkd_factor 1.0 \
  --debug_policy_io \
  --nodryrun
```

启动后的预期过程：

1. 程序等待 `/lowstate`、遥控器和G1-Comp第一帧反馈。
2. 自动进入ColdStart，身体缓慢移动到stand初始姿态。
3. ColdStart期间头部目标为 `yaw=0、pitch=0`。
4. 完成后终端反复提示：

   ```text
   ColdStartAgent done. Press 'R1' ...
   ```

5. 按 `R1`进入stand。
6. stand期间确认头部保持 `0°/0°`，没有抖动、反向或撞限位。
7. 支撑状态下按 `A`进入walk，但摇杆保持中位，只观察关节稳定性。
8. 按 `R1`返回stand。
9. 按 `L1`进入parkour。此时速度scale重置为0，并且
   `--parkour_freeze_head`使头部保持训练默认姿态：

   ```text
   yaw约0°
   pitch约-5.31°
   ```

10. 按 `R1`返回stand。

出现以下情况立即按 `R2`/`L2`或物理急停：

- 头部向错误方向快速运动；
- 头部撞到机械限位；
- 身体出现剧烈跳变或高频抖动；
- ColdStart目标明显异常；
- D455或头部反馈超时；
- 终端出现NaN、维度错误或模型配对错误。

通过后按 `R2`或 `L2`结束本轮，再按 `Ctrl+C`确认程序正常退出。

---

## 10. 支撑状态下解冻parkour头部

保持机器人支撑，不再使用 `--parkour_freeze_head`：

```bash
python scripts/g1_three_policy.py \
  --stand_logdir "$G1_STAND_MODEL" \
  --walk_logdir "$G1_WALK_MODEL" \
  --logdir "$G1_PARKOUR_MODEL" \
  --camera_serial "$G1_D455_SERIAL" \
  --gimbal \
  --parkour_blend_duration 1.0 \
  --startup_step_size 0.02 \
  --kpkd_factor 1.0 \
  --debug_policy_io \
  --nodryrun
```

依次执行：

1. 等ColdStart完成；
2. 按 `R1`进入stand，确认头部0°/0°；
3. 按 `L1`进入parkour，摇杆保持中位；
4. 观察1秒平滑接管过程；
5. 检查头部运动方向、幅度和反馈；
6. 按 `R1`返回stand，确认头部回到0°/0°。

重点核对：

- 模型命令yaw为正时，物理头部向右；
- 模型命令pitch为负时，物理头部向下；
- 反馈与实际运动同向；
- 头部不超过yaw约±50°、pitch约−20°到+85°；
- parkour切换没有突跳。

任何方向不对都应停止并重新检查第6节，不要直接交换DDS电机ID或临时改sign。

---

## 11. 落地低速测试

确认支撑测试全部通过后，才把机器人放到平整、空旷、摩擦正常的地面。

第一次落地仍建议使用：

```text
--parkour_blend_duration 1.0
--startup_step_size 0.02
--kpkd_factor 1.0
```

操作顺序：

1. 所有摇杆中位；
2. 启动server；
3. 启动three-policy `--nodryrun`；
4. 等ColdStart完成；
5. 按 `R1`进入stand，静置观察；
6. 按 `A`进入walk；
7. 左摇杆只给很小的前后量，确认29-DoF walk；
8. 松开摇杆并按 `R1`返回stand；
9. 按 `L1`进入parkour，此时parkour速度scale为0；
10. 先保持静止观察头部和身体；
11. 仅缓慢向前推动左摇杆 `ly`，逐渐增加parkour前进scale；
12. 松开摇杆，按 `R1`返回stand。

按键：

| 按键 | 功能 |
|---|---|
| `R1` | 切换到29-DoF stand |
| `A` | 切换到29-DoF walk |
| `L1` | 切换到31-DoF parkour，并把parkour速度scale重置为0 |
| `R2`或`L2` | 软件急停：关闭身体命令并通知server释放头部 |

---

## 12. 正式部署命令

两个终端都先重新执行第1节的变量设置。

### 终端A：G1-Comp server

```bash
cd "$G1_DEPLOY_REPO/g1_comp_servo_service"
source /opt/ros/foxy/setup.bash
source "$G1_UNITREE_SETUP"
export ROS_DOMAIN_ID=42

./build/main \
  --network "$G1_DDS_IFACE" \
  --domain "$ROS_DOMAIN_ID" \
  --serial "$G1_HEAD_PORT" \
  --config "$G1_DEPLOY_REPO/g1_comp_servo_service/config/config.yaml"
```

### 终端B：Three-policy

```bash
cd "$G1_DEPLOY_REPO"
source /opt/ros/foxy/setup.bash
source "$G1_UNITREE_SETUP"
export ROS_DOMAIN_ID=42

python scripts/g1_three_policy.py \
  --stand_logdir "$G1_STAND_MODEL" \
  --walk_logdir "$G1_WALK_MODEL" \
  --logdir "$G1_PARKOUR_MODEL" \
  --camera_serial "$G1_D455_SERIAL" \
  --gimbal \
  --parkour_blend_duration 1.0 \
  --startup_step_size 0.02 \
  --kpkd_factor 1.0 \
  --nodryrun
```

稳定运行多次后，才考虑把ColdStart参数调回默认值。首次部署不建议一开始使用默认
`startup_step_size=0.2` 和 `kpkd_factor=2.0`。

---

## 13. 运行中监控

可以在第三个终端检查：

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic hz /lowstate
ros2 topic hz /g1_comp_servo/state
ros2 topic echo --once /g1_comp_servo/state
```

部署代码包含以下保护：

- G1-Comp反馈超过1秒未更新：停止身体命令并释放头部；
- parkour期间D455深度超过1秒未更新：停止控制；
- server超过1秒没有收到头部命令：自动释放头部扭矩；
- G1-Comp server未发布第一帧反馈：真机控制不会进入ready状态；
- parkour模型、环境配置或ONNX维度不匹配：启动失败；
- `R2`或`L2`：软件急停。

---

## 14. 正确停机顺序

正常结束：

1. 松开所有摇杆；
2. 按 `R1`回到stand；
3. 按 `R2`或 `L2`触发软件急停；
4. 程序通常会自动退出；若仍在运行，再在three-policy终端按 `Ctrl+C`；
5. 等待看到：

   ```text
   Node shutdown complete.
   ```

6. 确认server终端打印头部释放信息；
7. 最后在server终端按 `Ctrl+C`；
8. 再关闭机器人或头部电源。

如果three-policy进程崩溃，server的1秒watchdog应自动释放头部。仍需使用物理急停处理
任何异常运动。

正常 `Ctrl+C`退出时，部署节点也会发送一次最终的身体motor-disable和头部
`mode=0`命令；但异常断电或进程被强制杀死时不能依赖该路径。

---

## 15. 常见故障

### 15.1 server打不开串口

```bash
ls -l "$G1_HEAD_PORT"
groups
fuser -v "$G1_HEAD_PORT"
```

检查串口路径、`dialout`权限以及是否有测试程序或DYNAMIXEL Wizard占用串口。

### 15.2 两个舵机没有反馈

- 确认供电；
- 确认USB转接器；
- 确认ID为0和1；
- 确认舵机通信速率为1 Mbps；
- 关闭server后再运行 `test_read_encoder`；
- 必要时使用文档提供的DYNAMIXEL Wizard检查，但不要同时运行server。

### 15.3 server正常，但ROS2看不到G1-Comp话题

```bash
echo "$ROS_DOMAIN_ID"
ip -br addr show "$G1_DDS_IFACE"
ros2 daemon stop
ros2 daemon start
ros2 topic list | rg g1_comp
```

重点检查server的 `--domain`、three-policy的 `ROS_DOMAIN_ID` 和网卡是否一致。

### 15.4 three-policy一直等待ready

分别检查：

```bash
ros2 topic hz /lowstate
ros2 topic hz /wirelesscontroller
ros2 topic hz /g1_comp_servo/state
```

任一关键话题没有数据，程序都会继续等待。

### 15.5 D455默认profile不支持

先确认序列号，然后降低到30 FPS：

```bash
--camera_width 848 --camera_height 480 --camera_fps 30
```

### 15.6 parkour模型维度错误

确认以下文件来自同一次训练导出：

```text
params/env.yaml
params/agent.yaml
exported/actor.onnx
exported/0-depth_encoder.onnx
```

新模型预期：

- actor输入：912维；
- actor输出：31维；
- `joint_pos`身体观测：29维；
- `joint_vel`身体观测：29维；
- 头部状态：独立2维 `camera_yaw_pitch`。

### 15.7 头部方向错误

立即急停，不要继续运行。重新执行：

1. 第4节encoder读取；
2. 第5节标定；
3. 第6节角度方向检查。

不要在未确认标定流程前修改 `direction`、交换ID或修改部署sign。

---

## 16. 明日上机最终检查表

- [ ] 机器人可靠支撑，物理急停可用
- [ ] G1-Comp线缆、供电、串口正常
- [ ] D455序列号确认
- [ ] `/lowstate`和遥控器话题正常
- [ ] server成功编译
- [ ] 被动encoder读取正常
- [ ] 三段式标定完成，`has_calibrate: 1`
- [ ] 被动角度方向与第6节表格一致
- [ ] server与ROS2 domain、网卡一致
- [ ] `/g1_comp_servo/state`持续更新
- [ ] three-policy dry-run无模型或相机错误
- [ ] 支撑状态、冻结头部的nodryrun通过
- [ ] 支撑状态、解冻头部的nodryrun通过
- [ ] stand/walk头部保持0°/0°
- [ ] parkour切换和头部方向正确
- [ ] 落地低速测试通过
- [ ] 操作人员熟悉 `R1/A/L1/R2/L2`
