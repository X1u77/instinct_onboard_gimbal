"""
舵机通讯协议层

实现协议帧的组包与解包

参考文档: https://wiki.fashionstar.com.hk/zh/servo/uart/protocols/uart-rs485-protocol/

-半双工异步串行通讯
-8位数据位, 1位停止位, 无奇偶校验
-Little Endian
-指令包头: 0x12 0x4C
-响应包头: 0x05 0x1C
-校验码: Σ(Byte[0..n]) % 256

"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple, List
import struct

import numpy as np


class ServoCommand(IntEnum):

    PING = 0x01                   # 通讯检测
    READ_DATA = 0x03              # 数据读取
    WRITE_DATA = 0x04             # 自定义配置参数
    DAMPING = 0x09                # 阻尼控制
    POSITION = 0x08               # 简易单圈角度控制
    READ_ANGLE = 0x0A             # 单圈当前角度读取
    POSITION_TIME = 0x0B          # 高级单圈角度控制(基于时间)
    POSITION_SPEED = 0x0C         # 高级单圈角度控制(基于速度)
    MULTI_POSITION = 0x0D         # 简易多圈角度控制
    MULTI_POSITION_TIME = 0x0E    # 高级多圈角度控制(基于时间)
    MULTI_POSITION_SPEED = 0x0F   # 高级多圈角度控制(基于速度)
    READ_MULTI_ANGLE = 0x10       # 多圈当前角度读取
    RESET_TURNS = 0x11            # 重置圈数
    SYNC_WRITE = 0x12             # 异步写入指令
    ASYNC_EXEC = 0x13             # 异步执行指令
    STOP = 0x18                   # 停止指令
    SYNC = 0x19                   # 同步指令
    MONITOR = 0x16                # 数据监控
    SET_ORIGIN = 0x17             # 设置原点


class ServoDataID(IntEnum):

    VOLTAGE = 1           # 电压 (mV)
    CURRENT = 2           # 电流 (mA)
    POWER = 3             # 功率 (mW)
    TEMPERATURE = 4       # 温度 (ADC)
    STATUS = 5            # 状态标志位
    CMD_RESPONSE = 33     # 指令响应开关
    SERVO_ID = 34         # 舵机ID
    BAUDRATE = 36         # 波特率配置
    STALL_PROTECT = 37    # 堵转保护开关
    STALL_POWER = 38      # 堵转功率上限
    VOLTAGE_MIN = 39      # 电压保护下限
    VOLTAGE_MAX = 40      # 电压保护上限
    TEMP_PROTECT = 41     # 温度保护值
    POWER_PROTECT = 42    # 功率保护值
    CURRENT_PROTECT = 43  # 电流保护值
    POWER_ON_LOCK = 46    # 舵机上电锁力开关
    ANGLE_LIMIT = 48      # 角度限制开关
    SOFT_START = 49       # 上电缓启动开关
    SOFT_START_TIME = 50  # 上电缓启动时间
    ANGLE_MAX = 51        # 舵机角度上限
    ANGLE_MIN = 52        # 舵机角度下限


class StopMode(IntEnum):

    RELEASE = 0x10        # 停止后释放锁力
    HOLD = 0x11           # 停止后保持锁力
    DAMPING = 0x12        # 停止后进入阻尼控制


@dataclass
class ServoStatus:

    servo_id: int = 0
    voltage_mv: int = 0           # 电压 (mV)
    current_ma: int = 0           # 电流 (mA)
    power_mw: int = 0             # 功率 (mW)
    temperature_adc: int = 0      # 温度 (ADC)
    status_flags: int = 0         # 状态标志位
    position: int = 0             # 当前位置 (0.1°)
    turns: int = 0                # 圈数


@dataclass
class ServoAngle:
  
    servo_id: int = 0
    position: int = 0            # 当前位置 (0.1°)
    turns: int = 0               # 圈数


class ServoProtocol:

    # 协议固定头部
    CMD_HEADER = bytes([0x12, 0x4C])    # 指令包头
    RSP_HEADER = bytes([0x05, 0x1C])     # 响应包头

    def __init__(self):
        pass

    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        return sum(data) % 256

    @staticmethod
    def _int16_to_bytes(value: int) -> bytes:
        return struct.pack('<h', value)

    @staticmethod
    def _uint16_to_bytes(value: int) -> bytes:
        return struct.pack('<H', value)

    @staticmethod
    def _int32_to_bytes(value: int) -> bytes:
        return struct.pack('<i', value)

    @staticmethod
    def _uint32_to_bytes(value: int) -> bytes:
        return struct.pack('<I', value)

    @staticmethod
    def _bytes_to_int16(data: bytes) -> int:
        return struct.unpack('<h', data)[0]

    @staticmethod
    def _bytes_to_uint16(data: bytes) -> int:
        return struct.unpack('<H', data)[0]

    @staticmethod
    def _bytes_to_int32(data: bytes) -> int:
        return struct.unpack('<i', data)[0]

    # ========== 组包函数  ==========

    def ping(self, servo_id: int) -> bytes:

        length = 0x01
        data = self.CMD_HEADER + bytes([ServoCommand.PING, length, servo_id])
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def read_angle(self, servo_id: int) -> bytes:
        length = 0x01
        data = self.CMD_HEADER + bytes([ServoCommand.READ_ANGLE, length, servo_id])
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def read_multi_angle(self, servo_id: int) -> bytes:
        length = 0x01
        data = self.CMD_HEADER + bytes([ServoCommand.READ_MULTI_ANGLE, length, servo_id])
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def position_control(self, servo_id: int, position: int,
                         time_ms: int, power_mw: int) -> bytes:
        length = 0x07
        data = (self.CMD_HEADER +
                bytes([ServoCommand.POSITION, length, servo_id]) +
                self._int16_to_bytes(position) +
                self._uint16_to_bytes(time_ms) +
                self._uint16_to_bytes(power_mw))
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def position_time_control(self, servo_id: int, position: int,
                              time_ms: int, accel_ms: int, decel_ms: int,
                              power_mw: int) -> bytes:
        length = 0x0B
        data = (self.CMD_HEADER +
                bytes([ServoCommand.POSITION_TIME, length, servo_id]) +
                self._int16_to_bytes(position) +
                self._uint16_to_bytes(time_ms) +
                self._uint16_to_bytes(accel_ms) +
                self._uint16_to_bytes(decel_ms) +
                self._uint16_to_bytes(power_mw))
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def position_speed_control(self, servo_id: int, position: int,
                               speed: int, accel_ms: int, decel_ms: int,
                               power_mw: int) -> bytes:
        length = 0x0B
        data = (self.CMD_HEADER +
                bytes([ServoCommand.POSITION_SPEED, length, servo_id]) +
                self._int16_to_bytes(position) +
                self._uint16_to_bytes(speed) +
                self._uint16_to_bytes(accel_ms) +
                self._uint16_to_bytes(decel_ms) +
                self._uint16_to_bytes(power_mw))
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def multi_position_control(self, servo_id: int, position: int,
                               time_ms: int, power_mw: int) -> bytes:
        length = 0x0B
        data = (self.CMD_HEADER +
                bytes([ServoCommand.MULTI_POSITION, length, servo_id]) +
                self._int32_to_bytes(position) +
                self._uint32_to_bytes(time_ms) +
                self._uint16_to_bytes(power_mw))
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def damping(self, servo_id: int, power_mw: int) -> bytes:
        length = 0x03
        data = (self.CMD_HEADER +
                bytes([ServoCommand.DAMPING, length, servo_id]) +
                self._uint16_to_bytes(power_mw))
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def stop(self, servo_id: int, mode: StopMode, power_mw: int) -> bytes:
        length = 0x04
        data = (self.CMD_HEADER +
                bytes([ServoCommand.STOP, length, servo_id, mode]) +
                self._uint16_to_bytes(power_mw))
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def read_data(self, servo_id: int, data_id: ServoDataID) -> bytes:
        length = 0x02
        data = (self.CMD_HEADER +
                bytes([ServoCommand.READ_DATA, length, servo_id, data_id]))
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def monitor(self, servo_id: int) -> bytes:
        length = 0x01
        data = self.CMD_HEADER + bytes([ServoCommand.MONITOR, length, servo_id])
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def set_origin(self, servo_id: int) -> bytes:
        length = 0x02
        data = (self.CMD_HEADER +
                bytes([ServoCommand.SET_ORIGIN, length, servo_id, 0x00]))
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def reset_turns(self, servo_id: int) -> bytes:
        length = 0x01
        data = self.CMD_HEADER + bytes([ServoCommand.RESET_TURNS, length, servo_id])
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    def sync_position_control(self, servo_commands: List[Tuple[int, int, int, int]]) -> bytes:
        cmd_id = ServoCommand.POSITION
        content_length = 0x07  # 每个子命令的长度
        count = len(servo_commands)

        # 构建内容部分
        # content = cmd_id + content_length + count + [servo1_data] + [servo2_data] + ...
        content = bytes([cmd_id, content_length, count])
        for servo_id, position, time_ms, power_mw in servo_commands:
            content += (bytes([servo_id]) +
                       self._int16_to_bytes(position) +
                       self._uint16_to_bytes(time_ms) +
                       self._uint16_to_bytes(power_mw))

        # length = content 的总字节数（不含自己）
        length = len(content)
        data = self.CMD_HEADER + bytes([ServoCommand.SYNC, length]) + content
        checksum = self.calculate_checksum(data)
        return data + bytes([checksum])

    # ========== 解包函数 ==========

    def parse_response(self, data: bytes) -> Optional[dict]:
        if len(data) < 5:
            return None

        if data[:2] != self.RSP_HEADER:
            return None

        cmd_id = data[2]
        length = data[3]
        servo_id = data[4]

        expected_len = length + 5  # header(2) + cmd_id(1) + length(1) + content + checksum(1)
        if len(data) < expected_len:
            return None

        payload = data[:length + 4]
        received_checksum = data[length + 4]
        calculated_checksum = self.calculate_checksum(payload)
        if received_checksum != calculated_checksum:
            return None

        result = {
            'cmd_id': cmd_id,
            'servo_id': servo_id,
            'length': length,
            'raw': data
        }

        content = data[4:length + 4]  # servo_id + content
        result['content'] = self._parse_content(cmd_id, content)

        return result

    def _parse_content(self, cmd_id: int, content: bytes) -> dict:
        result = {}

        if cmd_id == ServoCommand.PING:
            result['online'] = True

        elif cmd_id == ServoCommand.POSITION:
            if len(content) >= 2:
                result['result'] = content[1]  # 0x01: 成功, 0x00: 失败

        elif cmd_id == ServoCommand.READ_ANGLE:
            if len(content) >= 3:
                result['position'] = self._bytes_to_int16(content[1:3])

        elif cmd_id == ServoCommand.READ_MULTI_ANGLE:
            if len(content) >= 7:
                result['position'] = self._bytes_to_int32(content[1:5])
                result['turns'] = self._bytes_to_int16(content[5:7])

        elif cmd_id == ServoCommand.READ_DATA:
            if len(content) >= 3:
                data_id = content[0]
                data_content = content[1:]
                result['data_id'] = data_id
                if len(data_content) == 1:
                    result['value'] = struct.unpack('<b', bytes([data_content[0]]))[0]
                elif len(data_content) == 2:
                    result['value'] = self._bytes_to_uint16(data_content)

        elif cmd_id == ServoCommand.MONITOR:
            if len(content) >= 17:
                result['servo_id'] = content[0]
                result['voltage_mv'] = self._bytes_to_uint16(content[1:3])
                result['current_ma'] = self._bytes_to_uint16(content[3:5])
                result['power_mw'] = self._bytes_to_uint16(content[5:7])
                result['temperature_adc'] = self._bytes_to_uint16(content[7:9])
                result['status_flags'] = content[9]
                result['position'] = self._bytes_to_int32(content[10:14])
                result['turns'] = self._bytes_to_int16(content[14:16])

        return result

    def parse_angle_response(self, data: bytes) -> Optional[ServoAngle]:

        result = self.parse_response(data)
        if result is None:
            return None

        angle = ServoAngle()
        angle.servo_id = result['servo_id']

        content = result['content']
        if 'position' in content:
            angle.position = content['position']
        if 'turns' in content:
            angle.turns = content['turns']

        return angle

    def parse_status_response(self, data: bytes) -> Optional[ServoStatus]:
        
        result = self.parse_response(data)
        if result is None:
            return None

        content = result['content']
        if 'voltage_mv' not in content:
            return None

        status = ServoStatus()
        status.servo_id = content.get('servo_id', result['servo_id'])
        status.voltage_mv = content.get('voltage_mv', 0)
        status.current_ma = content.get('current_ma', 0)
        status.power_mw = content.get('power_mw', 0)
        status.temperature_adc = content.get('temperature_adc', 0)
        status.status_flags = content.get('status_flags', 0)
        status.position = content.get('position', 0)
        status.turns = content.get('turns', 0)

        return status

    # ========== 辅助函数 ==========

    @staticmethod
    def adc_to_temperature(adc: int) -> float:
        # 线性插值表
        temp_table = [
            (50, 1191), (51, 1164), (52, 1137), (53, 1110), (54, 1085),
            (55, 1059), (56, 1034), (57, 1010), (58, 986), (59, 963),
            (60, 941), (61, 918), (62, 897), (63, 876), (64, 855),
            (65, 835), (66, 815), (67, 796), (68, 777), (69, 759),
            (70, 741), (71, 723), (72, 706), (73, 689), (74, 673),
            (75, 657), (76, 642), (77, 627), (78, 612), (79, 598),
        ]

        if adc >= temp_table[0][1]:
            return temp_table[0][0]
        if adc <= temp_table[-1][1]:
            return temp_table[-1][0]

        for i in range(len(temp_table) - 1):
            temp_low, adc_low = temp_table[i]
            temp_high, adc_high = temp_table[i + 1]
            if adc_low >= adc >= adc_high:
                ratio = (adc_low - adc) / (adc_low - adc_high)
                return temp_low + ratio * (temp_high - temp_low)

        return 65.0  

    @staticmethod
    def degrees_to_protocol(degrees: float) -> int:

        return int(degrees * 10)

    @staticmethod
    def protocol_to_degrees(protocol_angle: int) -> float:
        return protocol_angle * 0.1

    @staticmethod
    def radians_to_protocol(radians: float) -> int:
        return int(np.degrees(radians) * 10)

    @staticmethod
    def protocol_to_radians(protocol_angle: int) -> float:
        return np.radians(protocol_angle * 0.1)
