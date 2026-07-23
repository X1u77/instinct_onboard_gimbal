"""ROS 2 bridge for Unitree's official G1-Comp servo service."""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import numpy as np


class G1CompServoServiceController:
    """Expose the official DDS servo service through the gimbal interface.

    The official ``g1_comp_servo_service`` owns the serial port, calibration,
    DYNAMIXEL SDK and motor enable state.  This class only publishes commands
    and caches the angle feedback exposed by that service.
    """

    def __init__(
        self,
        ros_node,
        cmd_topic: str = "/g1_comp_servo/cmd",
        state_topic: str = "/g1_comp_servo/state",
        use_dryrun: bool = False,
    ):
        self.ros_node = ros_node
        self.cmd_topic = cmd_topic
        self.state_topic = state_topic
        self.use_dryrun = bool(use_dryrun)
        self.pan_range = (-50.0, 50.0)
        self.tilt_range = (-20.0, 85.0)
        self.pan_servo_id = 0
        self.tilt_servo_id = 1
        self.baudrate = None  # Managed by the official server.

        self._lock = threading.Lock()
        self._angles = np.zeros(2, dtype=np.float64)
        self._last_state_time = 0.0
        self._connected = False
        self._publisher = None
        self._subscriber = None
        self._cmd_msg = None

    def connect(self) -> bool:
        if self.use_dryrun:
            self._connected = True
            return True

        from unitree_go.msg import MotorCmd, MotorCmds, MotorStates

        self._cmd_msg = MotorCmds()
        self._cmd_msg.cmds = [MotorCmd(), MotorCmd()]
        self._publisher = self.ros_node.create_publisher(MotorCmds, self.cmd_topic, 10)
        self._subscriber = self.ros_node.create_subscription(
            MotorStates,
            self.state_topic,
            self._state_callback,
            10,
        )
        self._connected = True
        return True

    def _state_callback(self, msg) -> None:
        if len(msg.states) < 2:
            self.ros_node.get_logger().warn(
                f"G1-Comp state contains {len(msg.states)} motors; expected 2."
            )
            return
        # The official service publishes q in degrees.
        with self._lock:
            self._angles[:] = (float(msg.states[0].q), float(msg.states[1].q))
            self._last_state_time = time.time()

    def get_gimbal_angles(self) -> Tuple[Optional[float], Optional[float]]:
        with self._lock:
            if not self.use_dryrun and self._last_state_time <= 0.0:
                return None, None
            return float(self._angles[0]), float(self._angles[1])

    def set_gimbal_angle(
        self,
        pan_degrees: float,
        tilt_degrees: float,
        **_kwargs,
    ) -> bool:
        targets = np.array(
            [
                np.clip(pan_degrees, *self.pan_range),
                np.clip(tilt_degrees, *self.tilt_range),
            ],
            dtype=np.float64,
        )
        if self.use_dryrun:
            with self._lock:
                self._angles[:] = targets
                self._last_state_time = time.time()
            return True
        if not self.is_connected():
            return False

        for index in range(2):
            self._cmd_msg.cmds[index].mode = 1
            self._cmd_msg.cmds[index].q = float(targets[index])
        self._publisher.publish(self._cmd_msg)
        return True

    def stop_all(self) -> bool:
        if self.use_dryrun or not self.is_connected():
            return True
        for command in self._cmd_msg.cmds:
            command.mode = 0
        self._publisher.publish(self._cmd_msg)
        return True

    def disconnect(self) -> None:
        if self._connected and not self.use_dryrun:
            self.stop_all()
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected
