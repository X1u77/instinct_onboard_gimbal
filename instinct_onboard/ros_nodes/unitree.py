import threading
import time

import numpy as np
import rclpy
from unitree_go.msg import WirelessController
from unitree_hg.msg import IMUState, LowCmd, LowState  # MotorState,; MotorCmd,

import instinct_onboard.robot_cfgs as robot_cfgs
from crc_module import get_crc
from instinct_onboard import utils
from instinct_onboard.ros_nodes.base import RealNode


class UnitreeNode(RealNode):
    """This is the implementation of the Unitree robot ROS interface.
    Supports both 29-DOF (standard G1) and 31-DOF (G1 with head gimbal).
    When using 31-DOF, set enable_gimbal=True to integrate UART-servo controlled
    head joints (head_yaw, head_pitch).
    """

    def __init__(
        self,
        low_state_topic: str = "/lowstate",
        low_cmd_topic: str = "/lowcmd",
        imu_state_topic: str = "/secondary_imu",
        joy_stick_topic: str = "/wirelesscontroller",
        enable_gimbal: bool = False,
        gimbal_serial_port: str = "/dev/ttyUSB0",
        gimbal_pan_servo_id: int = 0,
        gimbal_tilt_servo_id: int = 1,
        gimbal_pan_range: tuple = (-90.0, 90.0),
        gimbal_tilt_range: tuple = (-45.0, 45.0),
        **kwargs,
    ):
        self.enable_gimbal = enable_gimbal
        self.gimbal_serial_port = gimbal_serial_port
        self.gimbal_pan_servo_id = gimbal_pan_servo_id
        self.gimbal_tilt_servo_id = gimbal_tilt_servo_id
        self.gimbal_pan_range = gimbal_pan_range
        self.gimbal_tilt_range = gimbal_tilt_range
        self.gimbal = None
        self._gimbal_lock = threading.Lock()
        self._gimbal_reader_stop = threading.Event()
        self._gimbal_reader_thread = None
        self._gimbal_writer_stop = threading.Event()
        self._gimbal_writer_thread = None
        self._gimbal_target_cmd_deg = None
        self._gimbal_joint_pos = np.zeros(2, dtype=np.float32)
        self._gimbal_joint_vel = np.zeros(2, dtype=np.float32)
        self._gimbal_last_read_time = 0.0
        self._gimbal_last_cmd_deg = None
        self._gimbal_last_cmd_time = 0.0
        self._gimbal_cmd_min_delta_deg = 0.5
        self._gimbal_cmd_keepalive_interval = 0.5
        # ROS topic names
        self.low_state_topic = low_state_topic
        self.low_cmd_topic = (
            low_cmd_topic if not kwargs.get("dryrun", True) else low_cmd_topic + "_dryrun_" + str(np.random.randint(0, 65535))
        )
        self.imu_state_topic = imu_state_topic
        self.joy_stick_topic = joy_stick_topic
        super().__init__(node_name="unitree_node", **kwargs)

    def parse_config(self):
        super().parse_config()

        # load robot-specific configurations
        self.joint_map = getattr(robot_cfgs, self.robot_class_name).joint_map
        self.real_joint_names = getattr(robot_cfgs, self.robot_class_name).real_joint_names
        self.joint_signs = getattr(robot_cfgs, self.robot_class_name).joint_signs
        self.turn_on_motor_mode = getattr(robot_cfgs, self.robot_class_name).turn_on_motor_mode
        self.mode_pr = getattr(robot_cfgs, self.robot_class_name).mode_pr
        self._gimbal_joint_pos[:] = getattr(
            getattr(robot_cfgs, self.robot_class_name),
            "head_default_joint_pos",
            np.zeros(2, dtype=np.float32),
        )

        # Initialize gimbal controller for 31-DOF with head joints
        self._init_gimbal()

    def _init_gimbal(self):
        """Initialize the gimbal controller for head joints (only when enable_gimbal=True)."""
        if not self.enable_gimbal:
            return

        if self.NUM_JOINTS != 31:
            self.get_logger().warn(
                f"enable_gimbal=True but NUM_JOINTS={self.NUM_JOINTS}. "
                "Gimbal only works with 31-DOF. Ignoring gimbal initialization."
            )
            self.enable_gimbal = False
            return

        try:
            from instinct_onboard.servo import GimbalController
            self.gimbal = GimbalController(
                serial_port=self.gimbal_serial_port,
                pan_servo_id=self.gimbal_pan_servo_id,
                tilt_servo_id=self.gimbal_tilt_servo_id,
                pan_range=self.gimbal_pan_range,
                tilt_range=self.gimbal_tilt_range,
                use_dryrun=self.dryrun,
            )
            if not self.dryrun:
                if self.gimbal.connect():
                    self.get_logger().info(
                        f"Gimbal connected on {self.gimbal_serial_port}, "
                        f"pan servo={self.gimbal_pan_servo_id}, tilt servo={self.gimbal_tilt_servo_id}"
                    )
                    self._start_gimbal_reader()
                    self._start_gimbal_writer()
                else:
                    self.get_logger().error(f"Failed to connect gimbal on {self.gimbal_serial_port}")
                    self.gimbal = None
            else:
                self.get_logger().warn("Gimbal running in dryrun mode")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize gimbal: {e}")
            self.gimbal = None

    def _start_gimbal_reader(self):
        """Read UART servo feedback in the background so LowState callbacks stay fast."""
        if self.gimbal is None or self.dryrun:
            return
        if self._gimbal_reader_thread is not None and self._gimbal_reader_thread.is_alive():
            return
        self._gimbal_reader_stop.clear()
        self._gimbal_reader_thread = threading.Thread(
            target=self._gimbal_reader_loop,
            name="gimbal-reader",
            daemon=True,
        )
        self._gimbal_reader_thread.start()

    def _gimbal_reader_loop(self):
        read_period_s = 0.05
        while not self._gimbal_reader_stop.is_set():
            self._refresh_gimbal_feedback()
            time.sleep(read_period_s)

    def _refresh_gimbal_feedback(self):
        if self.gimbal is None:
            return
        try:
            pan_deg, tilt_deg = self.gimbal.get_gimbal_angles()
            if pan_deg is None or tilt_deg is None:
                return
            now = time.time()
            new_pos = np.array(
                [
                    np.deg2rad(pan_deg) * self.joint_signs[29],
                    np.deg2rad(tilt_deg) * self.joint_signs[30],
                ],
                dtype=np.float32,
            )
            with self._gimbal_lock:
                dt = max(now - self._gimbal_last_read_time, 1e-6) if self._gimbal_last_read_time else 0.0
                if dt > 0.0:
                    self._gimbal_joint_vel[:] = (new_pos - self._gimbal_joint_pos) / dt
                self._gimbal_joint_pos[:] = new_pos
                self._gimbal_last_read_time = now
        except Exception as e:
            self.get_logger().warn(f"Failed to read gimbal feedback: {e}")

    def _stop_gimbal_reader(self):
        self._gimbal_reader_stop.set()
        if self._gimbal_reader_thread is not None:
            self._gimbal_reader_thread.join(timeout=0.2)
            self._gimbal_reader_thread = None

    def _start_gimbal_writer(self):
        """Write UART servo commands in the background so body LowCmd is never blocked."""
        if self.gimbal is None or self.dryrun:
            return
        if self._gimbal_writer_thread is not None and self._gimbal_writer_thread.is_alive():
            return
        self._gimbal_writer_stop.clear()
        self._gimbal_writer_thread = threading.Thread(
            target=self._gimbal_writer_loop,
            name="gimbal-writer",
            daemon=True,
        )
        self._gimbal_writer_thread.start()

    def _gimbal_writer_loop(self):
        writer_period_s = 0.5
        while not self._gimbal_writer_stop.is_set():
            self._write_latest_gimbal_cmd()
            time.sleep(writer_period_s)

    def _write_latest_gimbal_cmd(self):
        if self.gimbal is None:
            return
        with self._gimbal_lock:
            if self._gimbal_target_cmd_deg is None:
                return
            cmd_deg = self._gimbal_target_cmd_deg.copy()

        try:
            now = time.time()
            if self._gimbal_last_cmd_deg is not None:
                max_delta = float(np.max(np.abs(cmd_deg - self._gimbal_last_cmd_deg)))
                if (
                    max_delta < self._gimbal_cmd_min_delta_deg
                    and now - self._gimbal_last_cmd_time < self._gimbal_cmd_keepalive_interval
                ):
                    return
            self.gimbal.set_gimbal_angle(
                pan_degrees=float(cmd_deg[0]),
                tilt_degrees=float(cmd_deg[1]),
            )
            self._gimbal_last_cmd_deg = cmd_deg
            self._gimbal_last_cmd_time = now
        except Exception as e:
            self.get_logger().warn(f"Failed to send gimbal command: {e}")

    def _stop_gimbal_writer(self):
        self._gimbal_writer_stop.set()
        if self._gimbal_writer_thread is not None:
            self._gimbal_writer_thread.join(timeout=0.2)
            self._gimbal_writer_thread = None

    def start_ros_handlers(self):
        """After initializing the env and policy, register ros related callbacks and topics"""
        super().start_ros_handlers()
        self.low_cmd_publisher = self.create_publisher(LowCmd, self.low_cmd_topic, 10)
        self.low_cmd_buffer = LowCmd()
        self.low_cmd_buffer.mode_pr = self.mode_pr

        # ROS subscribers
        self.low_state_subscriber = self.create_subscription(
            LowState, self.low_state_topic, self._low_state_callback, 10
        )
        self.torso_imu_subscriber = self.create_subscription(
            IMUState, self.imu_state_topic, self._torso_imu_state_callback, 10
        )
        self.joy_stick_subscriber = self.create_subscription(
            WirelessController, self.joy_stick_topic, self._joy_stick_callback, 10
        )
        self.get_logger().info(
            "ROS handlers started, waiting to receive critical low state and wireless controller messages."
        )
        if not self.dryrun:
            self.get_logger().warn(
                f"You are running the code in no-dryrun mode and publishing to '{self.low_cmd_topic}', Please keep"
                " safe."
            )
        else:
            self.get_logger().warn(
                f"You are publishing low cmd to '{self.low_cmd_topic}' because of dryrun mode, Please check and be"
                " safe."
            )
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.check_buffers_ready():
                break
        self.get_logger().info("All necessary buffers received, the robot is ready to go.")

    def check_buffers_ready(self):
        """Check if all the necessary buffers are ready to use. Only used at the the end of the start_ros_handlers."""
        buffer_ready = hasattr(self, "low_state_buffer") and self.joy_stick_data.lx is not None
        if self.imu_state_topic is not None:
            buffer_ready = buffer_ready and hasattr(self, "torso_imu_buffer")
        return buffer_ready

    """
    ROS callbacks and handlers that update the buffer
    """

    def _low_state_callback(self, msg):
        """store and handle proprioception data"""
        self.get_logger().info("Low state data received.", once=True)
        self.low_state_buffer = msg  # keep the latest low state
        self.low_cmd_buffer.mode_machine = msg.mode_machine

        # refresh joint_pos and joint_vel from Unitree LowState
        for sim_idx in range(self.NUM_JOINTS):
            real_idx = self.joint_map[sim_idx]
            if real_idx >= 0:
                # Standard Unitree motor
                self.joint_pos_[sim_idx] = self.low_state_buffer.motor_state[real_idx].q * self.joint_signs[sim_idx]
                self.joint_vel_[sim_idx] = self.low_state_buffer.motor_state[real_idx].dq * self.joint_signs[sim_idx]
            else:
                # Head gimbal feedback is cached by the UART reader thread.
                self.joint_pos_[sim_idx], self.joint_vel_[sim_idx] = self._read_gimbal_joint(sim_idx)

        # automatic safety check for Unitree motors only
        for sim_idx in range(self.NUM_JOINTS):
            real_idx = self.joint_map[sim_idx]
            if real_idx >= 0:
                if (
                    self.joint_pos_[sim_idx] > self.joint_pos_protect_high[sim_idx]
                    or self.joint_pos_[sim_idx] < self.joint_pos_protect_low[sim_idx]
                ):
                    self.get_logger().error(
                        f"Joint {sim_idx}(sim), {real_idx}(real) position out of range at"
                        f" {self.low_state_buffer.motor_state[real_idx].q}"
                    )
                    self.get_logger().error("The motors and this process shuts down.")
                    self._turn_off_motors()
                    raise SystemExit()

    def _read_gimbal_joint(self, sim_idx: int) -> tuple:
        """Read head joint angle from gimbal servo.
        sim_idx 29 -> head_yaw, sim_idx 30 -> head_pitch.
        Returns (position, velocity) in simulation coordinate system.
        """
        head_idx = sim_idx - 29
        with self._gimbal_lock:
            return float(self._gimbal_joint_pos[head_idx]), float(self._gimbal_joint_vel[head_idx])

    def _torso_imu_state_callback(self, msg):
        """store and handle torso imu data"""
        self.get_logger().info("Torso IMU data received.", once=True)
        self.torso_imu_buffer = msg

    def _joy_stick_callback(self, msg):
        self.get_logger().info("Wireless controller data received.", once=True)
        # fill the joy stick data
        self.joy_stick_data.A = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.A)
        self.joy_stick_data.B = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.B)
        self.joy_stick_data.X = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.X)
        self.joy_stick_data.Y = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.Y)
        self.joy_stick_data.start = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.start)
        self.joy_stick_data.select = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.select)
        self.joy_stick_data.L1 = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.L1)
        self.joy_stick_data.R1 = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.R1)
        self.joy_stick_data.L2 = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.L2)
        self.joy_stick_data.R2 = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.R2)
        self.joy_stick_data.up = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.up)
        self.joy_stick_data.down = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.down)
        self.joy_stick_data.left = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.left)
        self.joy_stick_data.right = bool(msg.keys & robot_cfgs.UnitreeWirelessButtons.right)
        # fill the joy stick data
        self.joy_stick_data.lx = msg.lx
        self.joy_stick_data.ly = msg.ly
        self.joy_stick_data.rx = msg.rx
        self.joy_stick_data.ry = msg.ry
        # left/right trigger is not available in Unitree Wireless Controller

        # refer to Unitree Remote Control data structure, msg.keys is a bit mask
        # 00000000 00000001 means pressing the 0-th button (R1)
        # 00000000 00000010 means pressing the 1-th button (L1)
        # 10000000 00000000 means pressing the 15-th button (left)
        if (msg.keys & robot_cfgs.UnitreeWirelessButtons.R2) or (
            msg.keys & robot_cfgs.UnitreeWirelessButtons.L2
        ):  # R2 or L2 is pressed
            self.get_logger().warn("R2 or L2 is pressed, the motors and this process shuts down.")
            self._turn_off_motors()
            raise SystemExit()

    """
    Refresh observation buffer and corresponding sub-functions
    NOTE: everything will be NON-batchwise. There is NO batch dimension in the observation.
    """

    def _get_quat_w_obs(self):
        """Get the quaternion in wxyz format from the torso IMU or low state buffer."""
        if hasattr(self, "torso_imu_buffer"):
            return np.array(self.torso_imu_buffer.quaternion, dtype=np.float32)
        else:
            return np.array(self.low_state_buffer.imu_state.quaternion, dtype=np.float32)

    def _get_base_ang_vel_obs(self):
        if hasattr(self, "torso_imu_buffer"):
            return np.array(self.torso_imu_buffer.gyroscope, dtype=np.float32)
        else:
            return np.array(self.low_state_buffer.imu_state.gyroscope, dtype=np.float32)

    def _get_projected_gravity_obs(self):
        if hasattr(self, "torso_imu_buffer"):
            quat_wxyz = np.quaternion(
                self.torso_imu_buffer.quaternion[0],
                self.torso_imu_buffer.quaternion[1],
                self.torso_imu_buffer.quaternion[2],
                self.torso_imu_buffer.quaternion[3],
            )
        else:
            quat_wxyz = np.quaternion(
                self.low_state_buffer.imu_state.quaternion[0],
                self.low_state_buffer.imu_state.quaternion[1],
                self.low_state_buffer.imu_state.quaternion[2],
                self.low_state_buffer.imu_state.quaternion[3],
            )
        return utils.quat_rotate_inverse(
            quat_wxyz,
            self.gravity_vec,
        ).astype(np.float32)

    """
    Control related functions
    """

    """
    Functions that actually publish the commands and take effect
    """

    def _publish_motor_cmd(
        self,
        target_joint_pos: np.array,  # shape (NUM_JOINTS,), in simulation order
        p_gains: np.ndarray,  # In the order of simulation joints, not real joints
        d_gains: np.ndarray,  # In the order of simulation joints, not real joints
    ):
        """Publish the joint commands to the robot motors in robot coordinates system.
        robot_coordinates_action: shape (NUM_JOINTS,), in simulation order.

        For 31-DOF: joints 0-28 go to Unitree LowCmd, joints 29-30 go to gimbal servo.
        """
        if np.isnan(target_joint_pos).any():
            self.get_logger().error("Robot coordinates action contain NaN, Skip sending the action to the robot.")
            return

        # Unitree motors (sim_index 0-28)
        for sim_idx in range(min(self.NUM_JOINTS, 29)):
            real_idx = self.joint_map[sim_idx]
            if real_idx >= 0:
                if not self.dryrun:
                    self.low_cmd_buffer.motor_cmd[real_idx].mode = self.turn_on_motor_mode[sim_idx]
                self.low_cmd_buffer.motor_cmd[real_idx].q = (target_joint_pos[sim_idx] * self.joint_signs[sim_idx]).item()
                self.low_cmd_buffer.motor_cmd[real_idx].dq = 0.0
                self.low_cmd_buffer.motor_cmd[real_idx].tau = 0.0
                self.low_cmd_buffer.motor_cmd[real_idx].kp = p_gains[sim_idx].item()
                self.low_cmd_buffer.motor_cmd[real_idx].kd = d_gains[sim_idx].item()

        # Head gimbal joints (sim_index 29-30), only present in the 31-DOF config.
        if self.NUM_JOINTS >= 31 and len(target_joint_pos) >= 31:
            self._publish_gimbal_cmd(target_joint_pos)

        self.low_cmd_buffer.crc = get_crc(self.low_cmd_buffer)
        self.low_cmd_publisher.publish(self.low_cmd_buffer)

    def _publish_gimbal_cmd(self, target_joint_pos: np.array):
        """Send head joint commands to the gimbal servo controller.
        Converts simulation coordinates (radians) to servo coordinates (degrees).
        """
        if self.NUM_JOINTS < 31 or len(target_joint_pos) < 31:
            return

        # head_yaw (sim_idx=29): sim radians -> servo degrees
        # joint_signs[29] = -1 (sim=left+ -> servo=right+)
        head_yaw_sim = target_joint_pos[29]
        head_yaw_servo_deg = np.rad2deg(head_yaw_sim) * self.joint_signs[29]

        # head_pitch (sim_idx=30): sim radians -> servo degrees
        # joint_signs[30] = +1 (same direction)
        head_pitch_sim = target_joint_pos[30]
        head_pitch_servo_deg = np.rad2deg(head_pitch_sim) * self.joint_signs[30]

        with self._gimbal_lock:
            self._gimbal_joint_pos[:] = [head_yaw_sim, head_pitch_sim]
            self._gimbal_joint_vel[:] = 0.0
            self._gimbal_target_cmd_deg = np.array(
                [head_yaw_servo_deg, head_pitch_servo_deg],
                dtype=np.float32,
            )

    def _turn_off_motors(self):
        """Turn off the motors"""
        for sim_idx in range(min(self.NUM_JOINTS, 29)):
            real_idx = self.joint_map[sim_idx]
            self.low_cmd_buffer.motor_cmd[real_idx].mode = 0x00
            self.low_cmd_buffer.motor_cmd[real_idx].q = 0.0
            self.low_cmd_buffer.motor_cmd[real_idx].dq = 0.0
            self.low_cmd_buffer.motor_cmd[real_idx].tau = 0.0
            self.low_cmd_buffer.motor_cmd[real_idx].kp = 0.0
            self.low_cmd_buffer.motor_cmd[real_idx].kd = 0.0
        self.low_cmd_buffer.crc = get_crc(self.low_cmd_buffer)
        self.low_cmd_publisher.publish(self.low_cmd_buffer)
        # Stop gimbal servos
        if self.gimbal is not None and not self.dryrun:
            self._stop_gimbal_writer()
            self._stop_gimbal_reader()
            self.gimbal.stop_all()

    def destroy_node(self):
        self._stop_gimbal_writer()
        self._stop_gimbal_reader()
        if self.gimbal is not None and not self.dryrun:
            self.gimbal.disconnect()
        super().destroy_node()