"""CRC helper for Unitree HG LowCmd messages.

This mirrors the Unitree SDK2 Python HG LowCmd packing and CRC32 algorithm
while accepting ROS message objects from ``unitree_hg.msg``.
"""

from __future__ import annotations

import struct
from typing import Any


LOWCMD_MOTOR_COUNT = 35


def _crc32_unitree(data: bytes) -> int:
    """Compute the CRC32 used by Unitree SDK2."""
    if len(data) % 4 != 0:
        raise ValueError("Unitree CRC input must be aligned to 32-bit words.")

    crc = 0xFFFFFFFF
    polynomial = 0x04C11DB7

    # Unitree SDK2 first interprets the packed little-endian byte buffer as
    # uint32 words, then feeds those words to the CRC routine.
    words = struct.unpack(f"<{len(data) // 4}I", data)
    for word in words:
        for bit in range(31, -1, -1):
            if crc & 0x80000000:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
            if word & (1 << bit):
                crc ^= polynomial
            crc &= 0xFFFFFFFF

    return crc


def _uint32(value: Any, default: int = 0) -> int:
    return int(value if value is not None else default) & 0xFFFFFFFF


def _uint8(value: Any, default: int = 0) -> int:
    return int(value if value is not None else default) & 0xFF


def _float32(value: Any, default: float = 0.0) -> float:
    return float(value if value is not None else default)


def _get_sequence_item(value: Any, index: int, default: int = 0) -> int:
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return int(value) if index == 0 else default
    if index < len(value):
        return int(value[index])
    return default


def _get_reserve_words(obj: Any, count: int) -> list[int]:
    reserve = getattr(obj, "reserve", None)
    if reserve is None:
        reserve = getattr(obj, "reserved", None)
    return [_uint32(_get_sequence_item(reserve, i, 0)) for i in range(count)]


def _pack_motor_cmd(motor: Any | None) -> bytes:
    if motor is None:
        return struct.pack("<B3x5fI", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

    reserve_word = _get_reserve_words(motor, 1)[0]
    return struct.pack(
        "<B3x5fI",
        _uint8(getattr(motor, "mode", 0)),
        _float32(getattr(motor, "q", 0.0)),
        _float32(getattr(motor, "dq", 0.0)),
        _float32(getattr(motor, "tau", 0.0)),
        _float32(getattr(motor, "kp", 0.0)),
        _float32(getattr(motor, "kd", 0.0)),
        reserve_word,
    )


def _pack_lowcmd_hg(msg: Any) -> bytes:
    """Pack a LowCmd in the same field order as Unitree SDK2 PackLowCmdHG."""
    buf = bytearray()
    buf.extend(
        struct.pack(
            "<2B2x",
            _uint8(getattr(msg, "mode_pr", 0)),
            _uint8(getattr(msg, "mode_machine", 0)),
        )
    )

    motors = getattr(msg, "motor_cmd", [])
    for i in range(LOWCMD_MOTOR_COUNT):
        motor = motors[i] if i < len(motors) else None
        buf.extend(_pack_motor_cmd(motor))

    for word in _get_reserve_words(msg, 4):
        buf.extend(struct.pack("<I", word))

    buf.extend(struct.pack("<I", _uint32(getattr(msg, "crc", 0))))
    return bytes(buf)


def get_crc(msg: Any) -> int:
    """Return the Unitree SDK2 CRC32 for a ``unitree_hg.msg.LowCmd`` object."""
    packed = _pack_lowcmd_hg(msg)
    return _crc32_unitree(packed[:-4])
