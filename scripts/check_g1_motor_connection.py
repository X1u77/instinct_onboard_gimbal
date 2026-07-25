#!/usr/bin/env python3
"""Read-only G1 motor-feedback checker.

This script only subscribes to /lowstate.  It never creates a LowCmd publisher,
does not change control mode, and cannot enable or move any motor.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from unitree_hg.msg import LowState


MOTOR_NAMES = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only G1 /lowstate motor feedback check")
    parser.add_argument("--seconds", type=float, default=3.0, help="sampling duration (default: 3)")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("g1_motor_connection_check")
    latest = {"msg": None, "count": 0}

    def callback(msg: LowState) -> None:
        latest["msg"] = msg
        latest["count"] += 1

    node.create_subscription(LowState, "/lowstate", callback, 10)
    deadline = time.monotonic() + max(args.seconds, 0.1)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = latest["msg"]
    if msg is None:
        print("FAIL: no /lowstate messages received. Check ROS_DOMAIN_ID and eth0 DDS connectivity.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(
        f"Received {latest['count']} /lowstate messages; "
        f"mode_pr={msg.mode_pr}, mode_machine={msg.mode_machine}."
    )
    print("index  name                    q(rad)     dq(rad/s) tau_est   temp(C)  voltage  motorstate")
    for index, name in enumerate(MOTOR_NAMES):
        motor = msg.motor_state[index]
        values = (motor.q, motor.dq, motor.tau_est, motor.vol)
        finite = all(math.isfinite(value) for value in values)
        temperatures = "/".join(str(value) for value in motor.temperature)
        status = "OK" if finite else "INVALID"
        print(
            f"{index:>5}  {name:<22} {motor.q:>8.3f} {motor.dq:>11.3f}"
            f" {motor.tau_est:>7.3f} {temperatures:>9} {motor.vol:>8.2f}"
            f" 0x{motor.motorstate:08x}  {status}"
        )

    print("\nNote: valid feedback proves DDS telemetry is present; motorstate bit meanings are firmware-specific.")
    print("Focus on real motor 18 (left_elbow) and 25 (right_elbow).")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
