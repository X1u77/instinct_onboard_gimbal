# g1_comp_servo_service

官方G1-Comp串口到DDS服务。部署端使用以下话题：

```text
rt/g1_comp_servo/cmd    unitree_go/MotorCmds
rt/g1_comp_servo/state  unitree_go/MotorStates
```

## 编译

需要先安装Unitree SDK2及其CycloneDDS依赖，并安装：

```bash
sudo apt install libyaml-cpp-dev libspdlog-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## 标定

```bash
./build/test_calibration \
  --serial /dev/ttyUSB0 \
  --config ./config/config.yaml
```

标定完成后确认 `has_calibrate: 1`。不要复制其他头部的两个calibration值。

## 启动服务

```bash
./build/main \
  --network eth0 \
  --domain 0 \
  --serial /dev/ttyUSB0 \
  --config ./config/config.yaml
```

将 `eth0` 替换为机器人DDS使用的网卡。`--domain`必须与three-policy进程的
`ROS_DOMAIN_ID`一致；省略时读取环境变量 `ROS_DOMAIN_ID`，未设置则使用0。

本目录中的服务保留官方角度换算、1 Mbps串口、ID 0/1及P/D增益。另修复了：

- 主程序重复创建串口对象且实际成员使用4.5 Mbps；
- 16位P/D增益错误使用4字节写；
- 状态发布和命令读取的数据竞争；
- 首次控制先上扭矩、后写目标导致的启动跳动风险；
- 命令超过1秒未更新后仍保持舵机使能；
- 配置文件路径硬编码为 `/home/unitree/...`。
- 读取测试会先关闭扭矩，标定完成后主动释放两个舵机。
