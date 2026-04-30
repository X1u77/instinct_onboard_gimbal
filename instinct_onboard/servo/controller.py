""" 舵机控制层 - 封装舵机控制逻辑 """

import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Callable
import numpy as np

from .protocol import (
    ServoProtocol, ServoCommand, ServoDataID, StopMode,
    ServoAngle, ServoStatus
)

# 串口默认值 (支持环境变量 PTZ_SERIAL_PORT)
DEFAULT_SERIAL_PORT = os.environ.get("PTZ_SERIAL_PORT", "/dev/ttyUSB0")

# 默认控制速度 (°/s)
DEFAULT_SPEED_DEG_S = 200.0


@dataclass
class ServoConfig:
    """舵机配置"""
    servo_id: int               # 舵机ID (0-254)
    name: str                  # 舵机名称 (如 "pan", "tilt")

    # 角度限制 (协议单位 0.1°)
    angle_min: int = -1800     # -180°
    angle_max: int = 1800      # +180°
    angle_home: int = 0        # 原点位置

    # 默认控制参数
    default_speed_deg_s: float = DEFAULT_SPEED_DEG_S  # 默认速度
    default_accel_ms: int = 50         # 默认加速时间
    default_decel_ms: int = 50         # 默认减速时间
    default_power_mw: int = 5000       # 默认功率限制

    # 方向修正
    direction_sign: float = 1.0        # 方向符号 (+1 或 -1)

    def __post_init__(self):
        assert 0 <= self.servo_id <= 254, f"Invalid servo_id: {self.servo_id}"
        assert self.angle_min <= self.angle_max, "angle_min must <= angle_max"


class ServoController:
    """舵机控制器基类"""

    def __init__(
        self,
        serial_port: str = DEFAULT_SERIAL_PORT,
        baudrate: int = 115200,
        timeout: float = 0.1,
        servo_configs: Optional[List[ServoConfig]] = None
    ):
        self.protocol = ServoProtocol()
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.timeout = timeout

        # 舵机配置
        self.servo_configs: Dict[int, ServoConfig] = {}
        if servo_configs:
            for config in servo_configs:
                self.servo_configs[config.servo_id] = config

        # 串口和锁
        self._serial = None
        self._lock = threading.Lock()
        self._is_connected = False

        # 状态缓存
        self._angle_cache: Dict[int, ServoAngle] = {}
        self._status_cache: Dict[int, ServoStatus] = {}

        # 回调函数
        self._angle_callbacks: List[Callable[[int, float], None]] = []
        self._status_callbacks: List[Callable[[int, ServoStatus], None]] = []

        # 指令间隔 (ms)
        self._cmd_interval_ms = 10.0  # 10ms

    def connect(self) -> bool:
        try:
            import serial
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                xonxoff=False,
                rtscts=False
            )
            # 清空接收缓冲区
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            self._is_connected = True
            return True
        except Exception as e:
            print(f"Failed to connect to serial port {self.serial_port}: {e}")
            self._is_connected = False
            return False

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._is_connected = False

    def is_connected(self) -> bool:
        return self._is_connected and self._serial is not None and self._serial.is_open

    def add_servo(self, config: ServoConfig):
        self.servo_configs[config.servo_id] = config

    def get_servo_config(self, servo_id: int) -> Optional[ServoConfig]:
        return self.servo_configs.get(servo_id)

    # ========== 基础通信方法 ==========

    def _send_raw(self, data: bytes):
        if not self.is_connected():
            raise RuntimeError("Serial port not connected")
        with self._lock:
            self._serial.write(data)
            self._serial.flush()

    def _recv_raw(self, num_bytes: int = 100) -> bytes:
        if not self.is_connected():
            raise RuntimeError("Serial port not connected")
        with self._lock:
            # 等待数据到达
            time.sleep(0.005)  # 等待5ms
            data = self._serial.read(num_bytes)
            return data

    def _send_and_recv(self, data: bytes, recv_len: int = 100) -> Optional[bytes]:
        if not self.is_connected():
            raise RuntimeError("Serial port not connected")
        with self._lock:
            self._serial.write(data)
            self._serial.flush()
            time.sleep(0.005)  # 等待响应
            data = self._serial.read(recv_len)
            return data if data else None

    def _wait_cmd_interval(self):
        time.sleep(self._cmd_interval_ms / 1000.0)

    # ========== 舵机操作方法 ==========

    def ping(self, servo_id: int) -> bool:
        cmd = self.protocol.ping(servo_id)
        response = self._send_and_recv(cmd)
        if response:
            result = self.protocol.parse_response(response)
            return result is not None
        return False

    def scan_servos(self, id_range: Tuple[int, int] = (0, 10)) -> List[int]:
        online_servos = []
        for servo_id in range(id_range[0], id_range[1] + 1):
            if self.ping(servo_id):
                online_servos.append(servo_id)
        return online_servos

    # ========== 角度控制方法 ==========

    def set_angle(
        self,
        servo_id: int,
        target_degrees: float,
        speed_deg_s: float = DEFAULT_SPEED_DEG_S,
        power_mw: Optional[int] = None
    ) -> bool:
        """
        设置舵机目标角度（基于速度控制）

        Args:
            servo_id: 舵机ID
            target_degrees: 目标角度（度）
            speed_deg_s: 速度（°/s），默认200°/s
            power_mw: 功率限制（mW）

        Returns:
            bool: 操作是否成功
        """
        if servo_id not in self.servo_configs:
            print(f"Warning: servo_id {servo_id} not configured")
            return False

        config = self.servo_configs[servo_id]

        # 角度转换和限制
        protocol_angle = int(target_degrees * 10 * config.direction_sign)
        protocol_angle = np.clip(protocol_angle, config.angle_min, config.angle_max)

        # 速度转换（°/s → 0.1°/s）
        speed_protocol = int(speed_deg_s * 10)

        # 使用默认值
        if power_mw is None:
            power_mw = config.default_power_mw

        try:
            # 使用基于速度的高级角度控制
            cmd = self.protocol.position_speed_control(
                servo_id, protocol_angle, speed_protocol,
                config.default_accel_ms, config.default_decel_ms, power_mw
            )
            self._send_raw(cmd)
            self._wait_cmd_interval()
            return True

        except Exception as e:
            print(f"Failed to set angle for servo {servo_id}: {e}")
            return False

    def set_angles(
        self,
        targets: Dict[int, float],
        speed_deg_s: float = DEFAULT_SPEED_DEG_S,
        power_mw: Optional[int] = None,
        sync: bool = True
    ) -> bool:
        """
        设置多个舵机目标角度（基于速度控制）

        Args:
            targets: {servo_id: target_degrees}
            speed_deg_s: 速度（°/s），默认200°/s
            power_mw: 功率限制（mW）
            sync: 是否使用同步指令

        Returns:
            bool: 操作是否成功
        """
        if sync and len(targets) > 1:
            # 使用同步指令（基于时间控制）
            commands = []
            for servo_id, target_degrees in targets.items():
                if servo_id not in self.servo_configs:
                    continue
                config = self.servo_configs[servo_id]

                protocol_angle = int(target_degrees * 10 * config.direction_sign)
                protocol_angle = np.clip(protocol_angle, config.angle_min, config.angle_max)
                p = power_mw if power_mw is not None else config.default_power_mw

                # 计算运行时间（ms）: time_ms = distance / speed * 1000
                # distance 单位是 0.1°，需要转换为度再计算
                distance_deg = abs(protocol_angle) / 10.0
                time_ms = int(distance_deg / speed_deg_s * 1000)
                time_ms = max(time_ms, 20)  # 最小20ms

                commands.append((servo_id, protocol_angle, time_ms, p))

            try:
                cmd = self.protocol.sync_position_control(commands)
                self._send_raw(cmd)
                self._wait_cmd_interval()
                return True
            except Exception as e:
                print(f"Failed to send sync command: {e}")
                return False
        else:
            # 逐个发送（基于速度控制）
            for servo_id, target_degrees in targets.items():
                self.set_angle(servo_id, target_degrees, speed_deg_s, power_mw)
            return True

    # ========== 角度读取方法 ==========

    def read_angle(self, servo_id: int, multi_turn: bool = False, retry: int = 3) -> Optional[float]:
        for attempt in range(retry):
            try:
                if multi_turn:
                    cmd = self.protocol.read_multi_angle(servo_id)
                else:
                    cmd = self.protocol.read_angle(servo_id)

                response = self._send_and_recv(cmd, recv_len=50)
                if response and len(response) >= 6:
                    angle = self.protocol.parse_angle_response(response)
                    if angle:
                        config = self.servo_configs.get(servo_id)
                        direction = config.direction_sign if config else 1.0
                        degrees = angle.position * 0.1 * direction
                        return degrees
                # 等待后重试
                time.sleep(0.02)
            except Exception as e:
                print(f"Failed to read angle for servo {servo_id} (attempt {attempt+1}): {e}")
        return None

    def read_angles(self, servo_ids: List[int]) -> Dict[int, float]:
        angles = {}
        for servo_id in servo_ids:
            angle = self.read_angle(servo_id)
            if angle is not None:
                angles[servo_id] = angle
        return angles

    # ========== 状态监控方法 ==========

    def read_status(self, servo_id: int) -> Optional[ServoStatus]:
        try:
            cmd = self.protocol.monitor(servo_id)
            response = self._send_and_recv(cmd)
            if response:
                return self.protocol.parse_status_response(response)
        except Exception as e:
            print(f"Failed to read status for servo {servo_id}: {e}")
        return None

    def read_temperature(self, servo_id: int) -> Optional[float]:
        try:
            cmd = self.protocol.read_data(servo_id, ServoDataID.TEMPERATURE)
            response = self._send_and_recv(cmd)
            if response:
                result = self.protocol.parse_response(response)
                if result and 'content' in result:
                    adc = result['content'].get('value', 0)
                    return self.protocol.adc_to_temperature(adc)
        except Exception as e:
            print(f"Failed to read temperature for servo {servo_id}: {e}")
        return None

    def read_power(self, servo_id: int) -> Optional[int]:
        try:
            cmd = self.protocol.read_data(servo_id, ServoDataID.POWER)
            response = self._send_and_recv(cmd)
            if response:
                result = self.protocol.parse_response(response)
                if result and 'content' in result:
                    return result['content'].get('value', 0)
        except Exception as e:
            print(f"Failed to read power for servo {servo_id}: {e}")
        return None

    # ========== 其他控制方法 ==========

    def damping(self, servo_id: int, power_mw: int = 500) -> bool:
        try:
            cmd = self.protocol.damping(servo_id, power_mw)
            self._send_raw(cmd)
            self._wait_cmd_interval()
            return True
        except Exception as e:
            print(f"Failed to set damping for servo {servo_id}: {e}")
            return False

    def stop(self, servo_id: int, mode: StopMode = StopMode.HOLD, power_mw: int = 0) -> bool:
        try:
            cmd = self.protocol.stop(servo_id, mode, power_mw)
            self._send_raw(cmd)
            self._wait_cmd_interval()
            return True
        except Exception as e:
            print(f"Failed to stop servo {servo_id}: {e}")
            return False

    def set_origin(self, servo_id: int) -> bool:
        try:
            cmd = self.protocol.set_origin(servo_id)
            self._send_raw(cmd)
            self._wait_cmd_interval()
            return True
        except Exception as e:
            print(f"Failed to set origin for servo {servo_id}: {e}")
            return False

    def reset_turns(self, servo_id: int) -> bool:
        try:
            cmd = self.protocol.reset_turns(servo_id)
            self._send_raw(cmd)
            self._wait_cmd_interval()
            return True
        except Exception as e:
            print(f"Failed to reset turns for servo {servo_id}: {e}")
            return False

    def stop_all(self) -> bool:
        success = True
        for servo_id in self.servo_configs.keys():
            if not self.stop(servo_id):
                success = False
        return success

    def damping_all(self, power_mw: int = 500) -> bool:
        success = True
        for servo_id in self.servo_configs.keys():
            if not self.damping(servo_id, power_mw):
                success = False
        return success

    # ========== 回调管理 ==========

    def add_angle_callback(self, callback: Callable[[int, float], None]):
        self._angle_callbacks.append(callback)

    def add_status_callback(self, callback: Callable[[int, ServoStatus], None]):
        self._status_callbacks.append(callback)

    def _notify_angle_change(self, servo_id: int, angle: float):
        for callback in self._angle_callbacks:
            try:
                callback(servo_id, angle)
            except Exception as e:
                print(f"Error in angle callback: {e}")

    def _notify_status_change(self, servo_id: int, status: ServoStatus):
        for callback in self._status_callbacks:
            try:
                callback(servo_id, status)
            except Exception as e:
                print(f"Error in status callback: {e}")


class GimbalController(ServoController):
    """云台控制器"""

    def __init__(
        self,
        serial_port: str = DEFAULT_SERIAL_PORT,
        baudrate: int = 115200,
        pan_servo_id: int = 0,
        tilt_servo_id: int = 1,
        pan_range: Tuple[float, float] = (-90.0, 90.0),
        tilt_range: Tuple[float, float] = (-45.0, 45.0),
        speed_deg_s: float = DEFAULT_SPEED_DEG_S,
        use_dryrun: bool = False,
    ):
        super().__init__(serial_port, baudrate)

        self.use_dryrun = use_dryrun
        self.pan_servo_id = pan_servo_id
        self.tilt_servo_id = tilt_servo_id
        self.speed_deg_s = speed_deg_s

        # 配置云台舵机
        self.add_servo(ServoConfig(
            servo_id=pan_servo_id,
            name="pan",
            angle_min=int(pan_range[0] * 10),
            angle_max=int(pan_range[1] * 10),
            default_speed_deg_s=speed_deg_s,
            default_power_mw=3000,
            direction_sign=1.0,
        ))

        self.add_servo(ServoConfig(
            servo_id=tilt_servo_id,
            name="tilt",
            angle_min=int(tilt_range[0] * 10),
            angle_max=int(tilt_range[1] * 10),
            default_speed_deg_s=speed_deg_s,
            default_power_mw=3000,
            direction_sign=1.0,
        ))

        # 缓存当前目标角度
        self._current_pan = 0.0
        self._current_tilt = 0.0

    def set_pan_angle(self, degrees: float) -> bool:
        self._current_pan = degrees
        return self.set_angle(self.pan_servo_id, degrees, self.speed_deg_s)

    def set_tilt_angle(self, degrees: float) -> bool:
        self._current_tilt = degrees
        return self.set_angle(self.tilt_servo_id, degrees, self.speed_deg_s)

    def set_gimbal_angle(
        self,
        pan_degrees: float,
        tilt_degrees: float,
        speed_deg_s: Optional[float] = None,
        sync: bool = True,
        time_ms: Optional[int] = None,
    ) -> bool:
        """设置云台角度（基于速度控制）"""
        self._current_pan = pan_degrees
        self._current_tilt = tilt_degrees
        if self.use_dryrun:
            return True
        speed = speed_deg_s if speed_deg_s is not None else self.speed_deg_s
        return self.set_angles({
            self.pan_servo_id: pan_degrees,
            self.tilt_servo_id: tilt_degrees
        }, speed, sync=sync)

    def get_pan_angle(self, multi_turn: bool = False) -> Optional[float]:
        return self.read_angle(self.pan_servo_id, multi_turn)

    def get_tilt_angle(self, multi_turn: bool = False) -> Optional[float]:
        return self.read_angle(self.tilt_servo_id, multi_turn)

    def get_gimbal_angles(self) -> Tuple[Optional[float], Optional[float]]:
        if self.use_dryrun:
            return self._current_pan, self._current_tilt
        return self.get_pan_angle(), self.get_tilt_angle()

    @property
    def current_pan(self) -> float:
        return self._current_pan

    @property
    def current_tilt(self) -> float:
        return self._current_tilt

    # ========== Dryrun 模式支持 ==========

    def connect(self) -> bool:
        if self.use_dryrun:
            self._is_connected = True
            return True
        return super().connect()

    def disconnect(self):
        if self.use_dryrun:
            self._is_connected = False
            return
        super().disconnect()

    def get_current_angle(self) -> Tuple[float, float]:
        if self.use_dryrun:
            return (self._current_pan, self._current_tilt)
        pan = self.get_pan_angle() or self._current_pan
        tilt = self.get_tilt_angle() or self._current_tilt
        return (pan, tilt)

    def set_target_angle(self, pan: float, tilt: float) -> bool:
        if self.use_dryrun:
            self._current_pan = pan
            self._current_tilt = tilt
            return True
        return self.set_gimbal_angle(pan, tilt)
