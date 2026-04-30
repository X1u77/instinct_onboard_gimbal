"""CRC module for Unitree LowCmd message checksum.

Implements the CRC16-CITT checksum used by Unitree SDK for LowCmd messages.
The polynomial is 0x1021 (CRC-CCITT), initialized to 0xFFFF.

Usage:
    from crc_module import get_crc
    low_cmd.crc = get_crc(low_cmd)
    publisher.publish(low_cmd)
"""

import struct
from typing import Any


def _crc16_ccitt(data: bytes) -> int:
    """Compute CRC16-CCITT (polynomial 0x1021) over a byte buffer.

    This is the standard CRC algorithm used by Unitree SDK for validating
    LowCmd messages sent over the network.

    Args:
        data: Raw byte buffer to compute CRC over.

    Returns:
        16-bit CRC value (unsigned).
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc


def get_crc(msg: Any) -> int:
    """Compute the CRC for a Unitree LowCmd message.

    The CRC is computed over the raw byte representation of the control data
    (everything before the crc field), serialized in little-endian format.

    LowCmd structure (from unitree_hg):
        uint8 mode_machine
        uint8 mode_pr
        uint8[10] padding (or similar reserved bytes)
        MotorCmd[30] motor_cmd
        uint32 crc (NOT included in CRC computation)

    Each MotorCmd:
        uint8 mode
        uint8 reserved
        float q
        float dq
        float ddq (often zeroed)
        float tau
        float kp
        float kd

    Args:
        msg: A unitree_hg.msg.LowCmd message object.

    Returns:
        16-bit CRC value as uint32.
    """
    buf = bytearray()

    # mode_machine (uint8)
    buf.extend(struct.pack("<B", msg.mode_machine))

    # mode_pr (uint8)
    buf.extend(struct.pack("<B", msg.mode_pr))

    # Reserved / padding bytes (typically 10 bytes in unitree_hg LowCmd)
    # Inspect motor_cmd[0] to detect the reserved field
    if len(msg.motor_cmd) > 0:
        first_motor = msg.motor_cmd[0]
        # Determine reserved field size by checking the struct layout
        motor0_size = 0
        if hasattr(first_motor, "reserved"):
            # reserved is uint8[2] or similar
            motor0_size += 2
        motor0_size += 4  # q (float)
        motor0_size += 4  # dq (float)
        motor0_size += 4  # ddq (float)
        motor0_size += 4  # tau (float)
        motor0_size += 4  # kp (float)
        motor0_size += 4  # kd (float)

        # Each motor: 26 bytes total (1 mode + 1 reserved + 6 floats)
        for i in range(30):
            motor = msg.motor_cmd[i]
            buf.extend(struct.pack("<B", motor.mode))
            if hasattr(motor, "reserved"):
                if isinstance(motor.reserved, (int, float)):
                    buf.extend(struct.pack("<B", int(motor.reserved)))
                elif isinstance(motor.reserved, (list, tuple, bytes)) and len(motor.reserved) >= 1:
                    buf.extend(struct.pack("<B", int(motor.reserved[0])))
                else:
                    buf.extend(struct.pack("<B", 0))
            else:
                buf.extend(struct.pack("<B", 0))
            buf.extend(struct.pack("<f", motor.q))
            buf.extend(struct.pack("<f", motor.dq))
            if hasattr(motor, "ddq"):
                buf.extend(struct.pack("<f", motor.ddq))
            else:
                buf.extend(struct.pack("<f", 0.0))
            buf.extend(struct.pack("<f", motor.tau))
            buf.extend(struct.pack("<f", motor.kp))
            buf.extend(struct.pack("<f", motor.kd))
    else:
        # No motors - padding only
        pass

    crc_val = _crc16_ccitt(bytes(buf))
    # Unitree SDK stores CRC as uint32 even though it's a 16-bit CRC
    return crc_val
