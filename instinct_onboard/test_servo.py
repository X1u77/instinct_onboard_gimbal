#!/usr/bin/env python3
"""
舵机调试测试脚本

使用方法:
    python3 test_servo.py --port /dev/ttyUSB0
    python3 test_servo.py --scan   
    python3 test_servo.py --dryrun  
    python3 instinct_onboard/test_servo.py --port xxx --test-angle 45 20
    python3 instinct_onboard/test_servo.py --port xxx --sweep
"""

import os
import sys
import time
import argparse

# 添加项目路径（向上找一级到项目根目录）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from instinct_onboard.servo.controller import GimbalController


def print_step(step: int, title: str):
    print(f"\n{'='*60}")
    print(f"  Step {step}: {title}")
    print('='*60)


def print_result(success: bool, message: str):
    symbol = "✅" if success else "❌"
    print(f"{symbol} {message}")


def scan_serial_ports():
    print_step(1, "扫描串口设备")

    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())

        if not ports:
            print_result(False, "未找到任何串口设备")
            return []

        print(f"找到 {len(ports)} 个串口设备:\n")
        for i, port in enumerate(ports, 1):
            print(f"  {i}. {port.device}")
            print(f"     描述: {port.description}")
            print(f"     硬件ID: {port.hwid}")
            print()

        return [p.device for p in ports]

    except ImportError:
        print_result(False, "请安装 pyserial: pip install pyserial")
        return []


def check_permissions(port: str):
    print_step(2, "检查串口权限")

    if not os.path.exists(port):
        print_result(False, f"串口 {port} 不存在")
        return False

    # 检查读权限
    has_read = os.access(port, os.R_OK)
    has_write = os.access(port, os.W_OK)

    print(f"  设备路径: {port}")
    print(f"  读权限: {'是' if has_read else '否'}")
    print(f"  写权限: {'是' if has_write else '否'}")

    if not has_read or not has_write:
        print_result(False, "权限不足")
        print(f"\n  解决方法:")
        print(f"    sudo chmod 666 {port}")
        print(f"    # 或将用户添加到 dialout 组:")
        print(f"    sudo usermod -a -G dialout $USER")
        return False

    print_result(True, "权限检查通过")
    return True


def test_connection(port: str, baudrate: int = 115200, dryrun: bool = False):
    """测试串口连接"""
    print_step(3, "测试串口连接")

    print(f"  串口: {port}")
    print(f"  波特率: {baudrate}")
    print(f"  模式: {'模拟' if dryrun else '实际连接'}")
    print()

    try:
        controller = GimbalController(
            serial_port=port,
            baudrate=baudrate,
            pan_range=(-90.0, 90.0),
            tilt_range=(-45.0, 45.0),
            use_dryrun=dryrun,
        )

        if dryrun:
            print_result(True, "模拟模式 - 代码逻辑测试通过")
            return controller

        # 尝试连接
        if controller.connect():
            print_result(True, f"成功连接到 {port}")
            return controller
        else:
            print_result(False, f"无法连接到 {port}")
            return None

    except Exception as e:
        print_result(False, f"连接失败: {e}")
        return None


def test_read_angle(controller: GimbalController, dryrun: bool = False):

    print_step(4, "读取当前角度")

    if controller is None and not dryrun:
        print_result(False, "控制器未初始化")
        return

    try:
        pan, tilt = controller.get_current_angle()
        print(f"  Pan (水平):  {pan:.2f}°")
        print(f"  Tilt (俯仰): {tilt:.2f}°")
        print_result(True, "读取角度成功")
    except Exception as e:
        print_result(False, f"读取角度失败: {e}")


def test_send_command(controller: GimbalController, pan: float = 0.0, tilt: float = 0.0, dryrun: bool = False):
    print_step(5, f"发送控制指令 (pan={pan}°, tilt={tilt}°)")

    if controller is None and not dryrun:
        print_result(False, "控制器未初始化")
        return

    try:
        controller.set_target_angle(pan, tilt)
        print_result(True, f"已发送目标角度: pan={pan}°, tilt={tilt}°")

        if not dryrun:
            # 等待一下让舵机响应
            time.sleep(0.5)
            pan_actual, tilt_actual = controller.get_current_angle()
            print(f"\n  实际角度: pan={pan_actual:.2f}°, tilt={tilt_actual:.2f}°")

    except Exception as e:
        print_result(False, f"发送指令失败: {e}")


def test_sweep(controller: GimbalController, dryrun: bool = False):
    print_step(6, "云台扫描测试")

    if controller is None and not dryrun:
        print_result(False, "控制器未初始化")
        return

    positions = [
        (0.0, 0.0),
        (30.0, 15.0),
        (-30.0, 15.0),
        (-30.0, -15.0),
        (30.0, -15.0),
        (0.0, 0.0),
    ]

    try:
        for i, (pan, tilt) in enumerate(positions):
            print(f"\n  [{i+1}/{len(positions)}] 移动到 pan={pan}°, tilt={tilt}°")
            controller.set_target_angle(pan, tilt)

            if not dryrun:
                time.sleep(1.0)
                pan_actual, tilt_actual = controller.get_current_angle()
                print(f"      当前位置: pan={pan_actual:.2f}°, tilt={tilt_actual:.2f}°")
            else:
                time.sleep(0.3)

        print_result(True, "扫描测试完成")

    except Exception as e:
        print_result(False, f"扫描测试失败: {e}")


def test_ping_all(controller: GimbalController):
    """Ping 所有舵机 ID"""
    print_step(6, "Ping 舵机扫描")

    if controller is None:
        print_result(False, "控制器未初始化")
        return

    print("  扫描 ID 范围: 0-20\n")

    online_servos = []

    for servo_id in range(21):
        try:
            online = controller.ping(servo_id)
            if online:
                online_servos.append(servo_id)
                print(f"  ✅ ID={servo_id} 在线")
            else:
                print(f"  ❌ ID={servo_id} 无响应")
        except Exception as e:
            print(f"  ❌ ID={servo_id} 错误: {e}")

    print(f"\n  找到 {len(online_servos)} 个在线舵机: {online_servos}")
    return online_servos


def cleanup(controller: GimbalController):

    print_step(7, "清理资源")

    if controller:
        try:
            controller.disconnect()
            print_result(True, "已断开连接")
        except Exception:
            pass

    print("\n" + "="*60)
    print("  测试完成!")
    print("="*60)


def main():
    parser = argparse.ArgumentParser(description="舵机调试测试脚本")
    parser.add_argument('--port', '-p', type=str,
                        help='串口设备路径，如 /dev/ttyUSB0')
    parser.add_argument('--scan', '-s', action='store_true',
                        help='只扫描串口，不连接')
    parser.add_argument('--baudrate', '-b', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('--dryrun', '-d', action='store_true',
                        help='模拟模式，不实际连接硬件')
    parser.add_argument('--test-angle', '-t', nargs=2, type=float,
                        metavar=('PAN', 'TILT'),
                        help='测试指定角度，如: --test-angle 30 15')
    parser.add_argument('--sweep', action='store_true',
                        help='运行扫描测试')
    parser.add_argument('--ping', action='store_true',
                        help='Ping 所有舵机 ID (0-10)')
    parser.add_argument('--read', '-r', action='store_true',
                        help='只读取当前角度，不发送控制指令')

    args = parser.parse_args()

    print("\n" + "="*60)
    print("        舵机调试测试脚本")
    print("="*60)

    # Step 0: 读取环境变量
    env_port = os.environ.get("PTZ_SERIAL_PORT")
    if env_port:
        print(f"\n 检测到环境变量 PTZ_SERIAL_PORT={env_port}")

    available_ports = scan_serial_ports()

    if args.scan:
        return

    if args.port:
        port = args.port
    elif env_port:
        port = env_port
    elif available_ports:
        port = available_ports[0]
        print(f"使用第一个可用串口: {port}")
    else:
        print("\n 未找到可用串口，且未指定串口路径")
        print("   请使用 --port 参数指定串口")
        return

    if not args.dryrun:
        if not check_permissions(port):
            response = input("\n是否继续? (y/N): ")
            if response.lower() != 'y':
                return
   
    controller = test_connection(port, args.baudrate, args.dryrun)

    if args.ping:
        test_ping_all(controller)
        cleanup(controller)
        return

    test_read_angle(controller, args.dryrun)

    # --read 只读取角度，不发送控制
    if args.read:
        cleanup(controller)
        return

   
    if args.test_angle:
        pan, tilt = args.test_angle
        test_send_command(controller, pan, tilt, args.dryrun)
    else:
        test_send_command(controller, 30.0, 30.0, args.dryrun)
   
    if args.sweep:
        test_sweep(controller, args.dryrun)

    if args.ping:
        test_ping_all(controller)

    cleanup(controller)


if __name__ == "__main__":
    main()
