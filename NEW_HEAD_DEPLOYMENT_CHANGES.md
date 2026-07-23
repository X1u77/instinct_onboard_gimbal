# 新头部 Three-Policy 部署修改概要

日期：2026-07-23

## 目标

部署组合保持为：

- stand：29-DoF旧模型；
- walk：29-DoF旧模型；
- parkour：Humanoid-Monument-Valley 中 `G1Comp`/D455新头部训练的31-DoF模型。

## 主要修改

1. 新增 `G1_31Dof_D455` 真机配置。
   - 保持29个身体关节顺序和 Unitree motor map 不变。
   - 新头部默认角度改为 `[0, -0.0926646] rad`。
   - 按官方server的实际角度公式将 head yaw/pitch 符号设为 `[-1, +1]`。
   - 同步新 URDF 机械限位、D455 FOV和深度范围。
   - D455辅助TF改为挂在运动的 `d455_link` 下。

2. Three-policy 入口改用新配置。
   - `g1_three_policy.py` 使用 `G1_31Dof_D455`。
   - D455默认 profile 改为 `848×480 @ 60 FPS`，序列号默认自动发现。
   - 增加D455相机profile参数；头部串口参数全部交给官方server。
   - 真机模式强制要求启用 gimbal，防止31-DoF策略在没有头部反馈/控制时运行。

3. 直接使用官方 `g1_comp_servo_service`。
   - 官方server独占串口并负责DYNAMIXEL SDK、标定换算、1 Mbps通信、ID 0/1、
     P/D增益和扭矩使能。
   - 部署端只通过 `rt/g1_comp_servo/cmd` 和 `rt/g1_comp_servo/state` DDS话题
     发送角度、接收反馈，不再自行读写DYNAMIXEL寄存器。
   - 按官方 `utilities.h` 的真实公式处理坐标：两个calibration都对应配置下限，
     因而yaw需要额外取反，pitch不取反。
   - 修复官方server中重复串口实例/错误波特率、16位增益使用4字节写、DDS数据竞争、
     首次上使能前未写目标、配置路径硬编码等问题。
   - 增加1秒命令watchdog；部署进程退出或DDS命令中断后，server自动释放头部扭矩。
   - 旧UART私有协议仍保留为 `legacy` 后端，不再由three-policy入口使用。

4. 保持29-DoF stand/walk兼容。
   - 两个29-DoF wrapper 仍只向模型提供29维身体观测和动作。
   - 输出继续扩展到31维。
   - 扩展出的两维不进入policy；stand/walk期间头部固定为
     `yaw=0、pitch=0 rad`。
   - 增加29-DoF actor输入/输出维度检查，防止误加载31-DoF模型或不配套的
     `env.yaml`。

5. 修正新31-DoF parkour观测。
   - 根据导出的 `env.yaml/SceneEntityCfg` 解析 joint selection 和顺序。
   - 新 `G1Comp` 模型的 `joint_pos`、`joint_vel` 均严格使用29个身体关节。
   - 修复旧代码把31维 joint position 输入新模型、造成后续观测整体错位的问题。
   - 增加 actor/depth encoder输入输出维度检查及新旧模型配对检查。

6. 修正深度图链路。
   - 使用节点实际相机分辨率/FPS，不再硬编码D435的 `480×270/60`。
   - 只修复零值或非有限的无效深度，不再误处理有效近距离像素。
   - 全空启动帧按最远距离处理。
   - 修复非零最小深度范围下的反归一化公式。
   - 降低相机延迟日志频率，并避免共享内存读到半帧。

7. 其他可靠性修复。
   - 修复 `kp=0` 时 torque clipping 除零。
   - 机器人关节符号数组改为实例副本，避免运行期校准污染全局配置。
   - Three-policy 循环实际发布31个关节的 `/joint_states`。
   - 忽略切换到当前策略的重复按键，避免按住按钮时每20 ms清空历史。
   - 切入29-DoF walk时清空其 body last-action，避免继承31-DoF策略的原始动作。
   - 真机云台连接失败时拒绝启动；启动时等待有效头部反馈，运行中反馈超过1秒未
     更新则停止电机命令。
   - parkour真机运行时D455深度帧超过1秒未更新则停止电机命令，避免长期使用
     旧深度图继续控制机器人。

## 验证结果

- `python -m compileall -q instinct_onboard scripts`：通过。
- robot profile数组长度、头部默认值、G1-Comp符号和限位静态断言：通过。
- G1-Comp DDS桥接消息构造、命令和反馈换算测试：通过。
- `git diff --check`：通过。

当前机器缺少ROS2、pyrealsense2、onnxruntime、yaml-cpp/spdlog开发头文件和
Unitree SDK2开发环境，未进行server完整编译、相机、串口、ONNX和真机联调。
首次真机前必须先执行官方标定和dry-run，并核对：

1. 手动向右转头时 yaw feedback 是否为正；
2. 手动向下低头时 pitch feedback 是否为负，抬头是否为正；
3. stand/walk时头部目标是否为 `yaw=0°、pitch=0°`；
4. 深度图有效范围是否为 `0.4–2.5 m`；
5. parkour raw head action为零时是否对应约 `yaw=0°、pitch=-5.31°`；
6. parkour ONNX启动时是否通过 `912维 actor input / 31维 action` 检查。

## G1-Comp资料依据

实现依据用户提供的 `G1_Comp.docx`、[宇树G1-Comp资料页](https://www.unitree.com/robocup/)、
[XL330控制表](https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/)、
[DYNAMIXEL SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
和 [DYNAMIXEL Wizard 2](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
官方文档及用户提供的 `g1_comp_servo_service` 源码。当前部署直接运行该server；
Python端只实现必要的ROS2/DDS桥接。
