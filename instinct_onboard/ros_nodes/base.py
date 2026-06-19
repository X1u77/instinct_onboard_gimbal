from abc import abstractmethod
from dataclasses import dataclass
import os
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import StaticTransformBroadcaster

from instinct_onboard import robot_cfgs


@dataclass
class JoyStickData:
    # None for not available
    lx: Optional[float] = None  # + for stick right, - for stick left
    ly: Optional[float] = None  # + for stick up, - for stick down
    rx: Optional[float] = None  # + for stick right, - for stick left
    ry: Optional[float] = None  # + for stick up, - for stick down
    left_trigger: Optional[float] = None  # + for trigger pressed, - for trigger released, but could be ranging (0, 1)
    right_trigger: Optional[float] = None  # + for trigger pressed, - for trigger released, but could be ranging (0, 1)

    # True for pressed, False for released
    up: Optional[bool] = None
    down: Optional[bool] = None
    left: Optional[bool] = None
    right: Optional[bool] = None
    A: Optional[bool] = None
    B: Optional[bool] = None
    X: Optional[bool] = None
    Y: Optional[bool] = None
    start: Optional[bool] = None
    select: Optional[bool] = None
    L1: Optional[bool] = None
    L2: Optional[bool] = None
    R1: Optional[bool] = None
    R2: Optional[bool] = None


class RealNode(Node):
    """This is the basic implementation of handling ROS messages matching the design of IsaacLab.
    It is designed to be used in the script directly to run the ONNX function. But please handle the
    impl of combining observations in the agent implementation.

    This also defines the most common features for each OEM node, and the agents should use this interface to interact with the robot.
    """

    def __init__(
        self,
        node_name: str,
        computer_clip_torque: bool = True,  # if True, the action will be clipped by torque limits
        joint_pos_protect_ratio: float = 1.5,  # if the joint_pos is out of the range of this ratio, the process will shutdown.
        kp_factor: float = 1.0,  # the factor to multiply the p_gain and clip the value to be in [0, 500]
        kd_factor: float = 1.0,  # the factor to multiply the d_gain
        kp_clip: float = 500,  # the maximum limit to the kp factor to prevent rediculious behavior.
        kd_clip: float = 20,  # the maximum limit to the kd factor to prevent rediculious behavior.
        torque_limits_ratio: float = 1.0,  # the factor to multiply the torque limits
        robot_class_name: str = None,  # the robot class name, used to get the robot configuration
        dryrun: bool = True,  # if True, the robot will not send commands to the real robot
    ):
        super().__init__(node_name)
        if robot_class_name is None:
            raise ValueError("robot_class_name must be provided")

        self.NUM_JOINTS = getattr(robot_cfgs, robot_class_name).NUM_JOINTS
        self.NUM_ACTIONS = getattr(robot_cfgs, robot_class_name).NUM_ACTIONS
        self.computer_clip_torque = computer_clip_torque
        self.joint_pos_protect_ratio = joint_pos_protect_ratio
        self.kp_factor = kp_factor
        self.kd_factor = kd_factor
        self.kp_clip = kp_clip
        self.kd_clip = kd_clip
        self.torque_limits_ratio = torque_limits_ratio
        self.robot_class_name = robot_class_name
        self.dryrun = dryrun
        self._debug_windows_enabled = False
        self._debug_depth_window_enabled = False
        self._debug_policy_window_enabled = False
        self._joint_tracking_enabled = False
        self._joint_tracking_logdir = None
        self._joint_tracking_interval = 0.02
        self._joint_tracking_show_plot = False
        self._joint_tracking_last_time = 0.0
        self._joint_tracking_start_time = None
        self._joint_tracking_records = []
        # This is a common joy stick data definition for multi-robot support.
        # Each OEM node should handle how to convert the raw joy stick data to this common definition.
        # Each agent should use this interface to acquire joy stick continuous values and button states.
        self._joy_stick_data = JoyStickData()

        self.parse_config()

    def parse_config(self):
        """Parse, set attributes from config dict, initialize buffers to speed up the computation"""

        self.up_axis_idx = 2  # 2 for z, 1 for y -> adapt gravity accordingly
        self.gravity_vec = np.zeros(3)
        self.gravity_vec[self.up_axis_idx] = -1

        self.torque_limits = (
            np.array(getattr(robot_cfgs, self.robot_class_name).torque_limits) * self.torque_limits_ratio
        )
        self.get_logger().info(f"Torque limits are set by ratio of : {self.torque_limits_ratio}")

        # buffers for observation output (in simulation order)
        self.joint_pos_ = np.zeros(
            self.NUM_JOINTS, dtype=np.float32
        )  # in robot urdf coordinate, but in simulation order. no offset subtracted
        self.joint_vel_ = np.zeros(self.NUM_JOINTS, dtype=np.float32)

        # action buffer
        self.action = np.zeros(self.NUM_ACTIONS, dtype=np.float32)

        # hardware related, in simulation order
        self.joint_signs = getattr(
            robot_cfgs, self.robot_class_name
        ).joint_signs  # in case of joint direction is different between sim and real
        self.sim_joint_names = getattr(robot_cfgs, self.robot_class_name).sim_joint_names
        self.joint_limits_high = np.array(getattr(robot_cfgs, self.robot_class_name).joint_limits_high)
        self.joint_limits_low = np.array(getattr(robot_cfgs, self.robot_class_name).joint_limits_low)
        joint_pos_mid = (self.joint_limits_high + self.joint_limits_low) / 2
        joint_pos_range = (self.joint_limits_high - self.joint_limits_low) / 2
        self.joint_pos_protect_high = joint_pos_mid + joint_pos_range * self.joint_pos_protect_ratio
        self.joint_pos_protect_low = joint_pos_mid - joint_pos_range * self.joint_pos_protect_ratio

    def start_ros_handlers(self):
        """Base method for initializing common ROS publishers.
        Derived classes should override this method to add their specific publishers/subscribers.
        """
        # Common publishers
        self.debug_msg_publisher = self.create_publisher(String, "/debug_msg", 10)
        self.action_publisher = self.create_publisher(Float32MultiArray, "/raw_actions", 10)

    @property
    def joy_stick_data(self) -> JoyStickData:
        return self._joy_stick_data

    def configure_debug_windows(
        self,
        *,
        enabled: bool = False,
        show_depth: bool = True,
        show_policy_io: bool = True,
    ):
        self._debug_windows_enabled = bool(enabled)
        self._debug_depth_window_enabled = bool(enabled and show_depth)
        self._debug_policy_window_enabled = bool(enabled and show_policy_io)

    def show_debug_depth_image(self, depth_image: np.ndarray):
        if not self._debug_depth_window_enabled:
            return
        try:
            import cv2
        except ImportError:
            self.get_logger().warn("OpenCV is not available, cannot show depth debug window.", once=True)
            return

        depth = np.asarray(depth_image, dtype=np.float32)
        finite = depth[np.isfinite(depth)]
        if finite.size == 0:
            return
        low = float(np.min(finite))
        high = float(np.max(finite))
        if high - low < 1e-6:
            high = low + 1.0
        normalized = np.clip((depth - low) / (high - low), 0.0, 1.0)
        depth_u8 = (normalized * 255.0).astype(np.uint8)
        colored = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
        cv2.putText(
            colored,
            f"depth min={low:.3f} max={high:.3f}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.imshow("policy depth image", colored)
        cv2.waitKey(1)

    def update_policy_io_debug_text(self, lines):
        if not self._debug_policy_window_enabled:
            return
        try:
            import cv2
        except ImportError:
            self.get_logger().warn("OpenCV is not available, cannot show policy I/O debug window.", once=True)
            return

        canvas = np.zeros((520, 1180, 3), dtype=np.uint8)
        y = 28
        for line in lines[:22]:
            cv2.putText(
                canvas,
                str(line)[:145],
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (230, 230, 230),
                1,
                cv2.LINE_AA,
            )
            y += 23
        cv2.imshow("policy input output summary", canvas)
        cv2.waitKey(1)

    def close_debug_windows(self):
        if not self._debug_windows_enabled:
            return
        try:
            import cv2
        except ImportError:
            return
        for window_name in ("policy depth image", "policy input output summary"):
            try:
                cv2.destroyWindow(window_name)
            except Exception:
                pass

    def configure_joint_tracking(
        self,
        *,
        enabled: bool = False,
        logdir: Optional[str] = None,
        interval: float = 0.02,
        show_plot: bool = False,
    ):
        self._joint_tracking_enabled = bool(enabled)
        self._joint_tracking_logdir = logdir
        self._joint_tracking_interval = max(float(interval), 0.0)
        self._joint_tracking_show_plot = bool(show_plot)
        self._joint_tracking_last_time = 0.0
        self._joint_tracking_start_time = None
        self._joint_tracking_records = []
        if self._joint_tracking_enabled and self._joint_tracking_logdir:
            os.makedirs(self._joint_tracking_logdir, exist_ok=True)

    def _record_joint_tracking(self, expected_joint_pos, commanded_joint_pos, raw_action):
        if not self._joint_tracking_enabled:
            return
        now = time.time()
        if now - self._joint_tracking_last_time < self._joint_tracking_interval:
            return
        if self._joint_tracking_start_time is None:
            self._joint_tracking_start_time = now
        self._joint_tracking_last_time = now
        self._joint_tracking_records.append(
            {
                "time": now - self._joint_tracking_start_time,
                "expected_joint_pos": np.asarray(expected_joint_pos, dtype=np.float32).copy(),
                "commanded_joint_pos": np.asarray(commanded_joint_pos, dtype=np.float32).copy(),
                "actual_joint_pos": np.asarray(self.joint_pos_, dtype=np.float32).copy(),
                "raw_action": np.asarray(raw_action, dtype=np.float32).copy(),
            }
        )

    def finalize_joint_tracking(self):
        if not self._joint_tracking_enabled or not self._joint_tracking_records:
            return None
        logdir = self._joint_tracking_logdir or os.path.join(os.getcwd(), "joint_tracking_logs")
        os.makedirs(logdir, exist_ok=True)
        time_data = np.asarray([record["time"] for record in self._joint_tracking_records], dtype=np.float32)
        expected = np.stack([record["expected_joint_pos"] for record in self._joint_tracking_records])
        commanded = np.stack([record["commanded_joint_pos"] for record in self._joint_tracking_records])
        actual = np.stack([record["actual_joint_pos"] for record in self._joint_tracking_records])
        raw_action = np.stack([record["raw_action"] for record in self._joint_tracking_records])
        npz_path = os.path.join(logdir, "joint_tracking.npz")
        np.savez_compressed(
            npz_path,
            time=time_data,
            expected_joint_pos=expected,
            commanded_joint_pos=commanded,
            actual_joint_pos=actual,
            raw_action=raw_action,
            joint_names=np.asarray(self.sim_joint_names),
        )

        plot_path = os.path.join(logdir, "joint_tracking.png")
        try:
            import matplotlib
            if not self._joint_tracking_show_plot:
                matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            cols = 4
            rows = int(np.ceil(self.NUM_JOINTS / cols))
            fig, axes = plt.subplots(rows, cols, figsize=(cols * 4.2, rows * 2.5), sharex=True)
            axes = np.asarray(axes).reshape(-1)
            for joint_id in range(self.NUM_JOINTS):
                ax = axes[joint_id]
                error = expected[:, joint_id] - actual[:, joint_id]
                ax.plot(time_data, expected[:, joint_id], label="expected", linewidth=1.0)
                ax.plot(time_data, actual[:, joint_id], label="actual", linewidth=1.0)
                ax.plot(time_data, error, label="error", linewidth=0.8, alpha=0.8)
                title = self.sim_joint_names[joint_id]
                ax.set_title(f"{joint_id}: {title}", fontsize=8)
                ax.grid(True, linewidth=0.3, alpha=0.5)
            for ax in axes[self.NUM_JOINTS :]:
                ax.axis("off")
            axes[0].legend(fontsize=7)
            fig.suptitle("Joint tracking: policy target vs motor feedback", fontsize=14)
            fig.supxlabel("time [s]")
            fig.supylabel("position [rad]")
            fig.tight_layout()
            fig.savefig(plot_path, dpi=160)
            if self._joint_tracking_show_plot:
                plt.show()
            plt.close(fig)
        except Exception as exc:
            self.get_logger().warn(f"Failed to create joint tracking plot: {exc}")
            plot_path = None

        self.get_logger().info(f"Joint tracking data saved to {npz_path}.")
        if plot_path:
            self.get_logger().info(f"Joint tracking plot saved to {plot_path}.")
        return plot_path or npz_path

    def publish_auxiliary_static_transforms(self, transform_field_name: str):
        """Publish some additional static transforms that are not part of the robot model.
        Args:
            transform_field_name: The field name in the robot_cfg of the given robot class. The transform data should
                be a dictionary with the following keys:
                    - translation: (x, y, z)
                    - rotation: (w, x, y, z)
                    - parent_frame: the frame id of the parent frame
                    - child_frame: the frame id of the child frame
        """
        if not hasattr(self, "static_tf_broadcaster"):
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        robot_transform_data = getattr(getattr(robot_cfgs, self.robot_class_name), transform_field_name)
        t.header.frame_id = robot_transform_data["parent_frame"]
        t.child_frame_id = robot_transform_data["child_frame"]
        t.transform.translation.x = robot_transform_data["translation"][0]
        t.transform.translation.y = robot_transform_data["translation"][1]
        t.transform.translation.z = robot_transform_data["translation"][2]
        t.transform.rotation.w = robot_transform_data["rotation"][0]
        t.transform.rotation.x = robot_transform_data["rotation"][1]
        t.transform.rotation.y = robot_transform_data["rotation"][2]
        t.transform.rotation.z = robot_transform_data["rotation"][3]
        self.static_tf_broadcaster.sendTransform(t)

    """
    Get observation term from the corresponding buffers
    NOTE: everything will be NON-batchwise. There is NO batch dimension in the observation.
    """

    def _get_joint_pos_obs(self):
        return self.joint_pos_  # shape (NUM_JOINTS,)

    def _get_joint_vel_obs(self):
        return self.joint_vel_  # shape (NUM_JOINTS,)

    def _get_joint_vel_rel_obs(self):
        """Get the joint velocity relative to the default joint velocity
        TODO: Get the default joint velocity from the configuration and update it in parse_config
        """
        return self.joint_vel_ - np.zeros(self.NUM_JOINTS, dtype=np.float32)  # shape (NUM_JOINTS,)

    def _get_last_action_obs(self):
        return self.action  # shape (NUM_ACTIONS,)

    """
    Functions that actually publish the commands and take effect
    """

    def clip_by_torque_limit(
        self,
        target_joint_pos,
        p_gains: np.ndarray = 0.0,
        d_gains: np.ndarray = 0.0,
    ):
        """Different from simulation, we reverse the process and clip the target position directly,
        so that the PD controller runs in robot but not our script.
        """
        p_limits_low = (-self.torque_limits) + d_gains * self.joint_vel_
        p_limits_high = (self.torque_limits) + d_gains * self.joint_vel_
        action_low = (p_limits_low / p_gains) + self.joint_pos_
        action_high = (p_limits_high / p_gains) + self.joint_pos_

        return np.clip(target_joint_pos, action_low, action_high)

    def send_action(
        self,
        action: np.array,
        action_offset: np.array = 0.0,
        action_scale: np.ndarray = 1.0,
        p_gains: np.ndarray = 0.0,
        d_gains: np.ndarray = 0.0,
    ):
        """Send the action to the robot motors, which does the preprocessing
        just like env.step in simulation.
        However, since this process only controls one robot, the action is not batched.
        NOTE: when switching between agents, the last_action term should be shared between agents.
        Thus, the ros node has to update the action buffer
        """
        action = np.asarray(action, dtype=np.float32)
        if not np.isfinite(action).all():
            self.get_logger().error("Actions contain NaN or Inf, Skip sending the action to the robot.")
            safe_action = np.nan_to_num(action, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
            self.action[:] = safe_action
            self.action_publisher.publish(Float32MultiArray(data=safe_action.tolist()))
            return

        # NOTE: Only calling this function currently will update self.actions for self._get_last_action_obs
        self.action[:] = action
        self.action_publisher.publish(Float32MultiArray(data=action.tolist()))
        action_scaled = action * action_scale
        target_joint_pos = action_scaled + action_offset
        expected_joint_pos = target_joint_pos.copy()
        p_gains = np.clip(p_gains * self.kp_factor, 0.0, self.kp_clip)
        d_gains = np.clip(d_gains * self.kd_factor, 0.0, self.kd_clip)
        if self.computer_clip_torque:
            target_joint_pos = self.clip_by_torque_limit(
                target_joint_pos,
                p_gains=p_gains,
                d_gains=d_gains,
            )
        self._record_joint_tracking(expected_joint_pos, target_joint_pos, action)
        self._publish_motor_cmd(target_joint_pos, p_gains=p_gains, d_gains=d_gains)

    @abstractmethod
    def _publish_motor_cmd(self, target_joint_pos: np.array, p_gains: np.array, d_gains: np.array):
        """Publish the joint commands to the robot motors in robot coordinates system.
        robot_coordinates_action: shape (NUM_JOINTS,), in simulation order.
        """
        pass

    @abstractmethod
    def _turn_off_motors(self):
        """Turn off the motors"""
        pass
