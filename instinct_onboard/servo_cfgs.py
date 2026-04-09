""" 舵机云台配置文件 """

import os
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional


@dataclass
class ServoSpec:
    servo_id: int                      # 舵机ID
    name: str                          # 舵机名称

    angle_min: int = -1800             # -180°
    angle_max: int = 1800              # +180°
    angle_home: int = 0                # 原点位置

    baudrate: int = 115200           

    default_move_time_ms: int = 500    # 运动时间
    default_power_mw: int = 5000       # 功率限制 (mW)
    accel_ms: int = 100                # 加速时间 (ms)
    decel_ms: int = 100                # 减速时间 (ms)

    # 与机器人坐标系的映射
    sim_joint_name: Optional[str] = None   
    direction_sign: float = 1.0        # 方向符号 (+1 或 -1)


class PTZServoConfig:

    NUM_SERVOS = 2

    PAN_SERVO = ServoSpec(
        servo_id=0,
        name="pan",
        angle_min=-900,     # -90° 左
        angle_max=900,      # +90° 右
        angle_home=0,      
        default_move_time_ms=200,  
        default_power_mw=3000,
        sim_joint_name="pan_joint",
        direction_sign=1.0,
    )

    TILT_SERVO = ServoSpec(
        servo_id=1,
        name="tilt",
        angle_min=-450,     # -45° 低
        angle_max=450,      # +45° 抬
        angle_home=0,      
        default_move_time_ms=200,
        default_power_mw=3000,
        sim_joint_name="tilt_joint",
        direction_sign=1.0,
    )

    SERVOS = [PAN_SERVO, TILT_SERVO]

    DEFAULT_SERIAL_PORT = os.environ.get("PTZ_SERIAL_PORT", "/dev/ttyUSB0")
    DEFAULT_SERIAL_TIMEOUT = 0.1  

    PROTOCOL_HEADER = bytes([0x12, 0x4C])     # 指令包头
    RESPONSE_HEADER = bytes([0x05, 0x1C])     # 响应包头

    CMD_INTERVAL_MS = 10  

    DEG_TO_PROTOCOL = 10.0    
    PROTOCOL_TO_DEG = 0.1     

    @classmethod
    def protocol_to_radians(cls, protocol_angle: int, servo: ServoSpec) -> float:
        return np.deg2rad(protocol_angle * 0.1) * servo.direction_sign

    @classmethod
    def radians_to_protocol(cls, radians: float, servo: ServoSpec) -> int:
        angle_deg = np.rad2deg(radians) * servo.direction_sign
        return int(np.clip(angle_deg * 10, servo.angle_min, servo.angle_max))

    @classmethod
    def get_servo_by_id(cls, servo_id: int) -> Optional[ServoSpec]:
        for servo in cls.SERVOS:
            if servo.servo_id == servo_id:
                return servo
        return None

    @classmethod
    def get_servo_by_name(cls, name: str) -> Optional[ServoSpec]:
        for servo in cls.SERVOS:
            if servo.name == name:
                return servo
        return None

    @classmethod
    def to_controller_configs(cls) -> List[Dict]:

        from instinct_onboard.servo import ServoConfig

        configs = []
        for servo in cls.SERVOS:
            configs.append(ServoConfig(
                servo_id=servo.servo_id,
                name=servo.name,
                angle_min=servo.angle_min,
                angle_max=servo.angle_max,
                angle_home=servo.angle_home,
                default_move_time_ms=servo.default_move_time_ms,
                default_accel_ms=servo.accel_ms,
                default_decel_ms=servo.decel_ms,
                default_power_mw=servo.default_power_mw,
                direction_sign=servo.direction_sign,
            ))
        return configs


class G1WithPTZConfig:
    """Unitree G1 + 云台云台的整体配置"""
 
    G1 = "G1_29Dof_TorsoBase"

    PTZ = PTZServoConfig

    GIMBAL_BASE_TRANSFORM = {
        "translation": (0.0, 0.0, 0.35),  
        "rotation": (1.0, 0.0, 0.0, 0.0),  
        "parent_frame": "head_link",
        "child_frame": "gimbal_base_link",
    }

    GIMBAL_JOINT_NAMES = ["pan_joint", "tilt_joint"]

