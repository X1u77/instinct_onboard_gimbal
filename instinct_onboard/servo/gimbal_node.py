""" 云台舵机ROS节点 """

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

from std_msgs.msg import Float32MultiArray, Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from .controller import GimbalController, ServoController
from .protocol import ServoStatus
from ..servo_cfgs import PTZServoConfig


class GimbalNode(Node):

    def __init__(self, node_name: str = "gimbal_node"):
        super().__init__(node_name)

        # ========== 参数声明 (使用 PTZServoConfig 默认值) ==========
        self.declare_parameter('serial_port', PTZServoConfig.DEFAULT_SERIAL_PORT)
        self.declare_parameter('baudrate', PTZServoConfig.PAN_SERVO.baudrate)
        self.declare_parameter('pan_servo_id', PTZServoConfig.PAN_SERVO.servo_id)
        self.declare_parameter('tilt_servo_id', PTZServoConfig.TILT_SERVO.servo_id)
        self.declare_parameter('pan_range', [PTZServoConfig.PAN_SERVO.angle_min / 10.0, PTZServoConfig.PAN_SERVO.angle_max / 10.0])
        self.declare_parameter('tilt_range', [PTZServoConfig.TILT_SERVO.angle_min / 10.0, PTZServoConfig.TILT_SERVO.angle_max / 10.0])
        self.declare_parameter('update_rate', 10.0)   # 状态发布频率
        self.declare_parameter('read_rate', 20.0)     # 角度读取频率
        self.declare_parameter('use_dryrun', True)
        self.declare_parameter('use_sync_control', True)  # 同步控制

        self._load_params()

        # ========== 初始化控制器 ==========
        self._init_controller()

        # ========== ROS发布者 ==========
        self._init_publishers()

        # ========== ROS订阅者 ==========
        self._init_subscribers()

        # ========== 定时器 ==========
        self._init_timers()

        # ========== 状态变量 ==========
        self._current_pan = 0.0
        self._current_tilt = 0.0
        self._target_pan = 0.0
        self._target_tilt = 0.0
        self._is_healthy = False
        self._last_read_time = 0.0

        self.get_logger().info(f"GimbalNode initialized (dryrun={self.use_dryrun})")

    def _load_params(self):
       
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.pan_servo_id = self.get_parameter('pan_servo_id').value
        self.tilt_servo_id = self.get_parameter('tilt_servo_id').value
        self.pan_range = self.get_parameter('pan_range').value
        self.tilt_range = self.get_parameter('tilt_range').value
        self.update_rate = self.get_parameter('update_rate').value
        self.read_rate = self.get_parameter('read_rate').value
        self.use_dryrun = self.get_parameter('use_dryrun').value
        self.use_sync_control = self.get_parameter('use_sync_control').value

    def _init_controller(self):
       
        try:
            self.controller = GimbalController(
                serial_port=self.serial_port,
                baudrate=self.baudrate,
                pan_servo_id=self.pan_servo_id,
                tilt_servo_id=self.tilt_servo_id,
                pan_range=tuple(self.pan_range),
                tilt_range=tuple(self.tilt_range),
            )

            if not self.use_dryrun:
                if self.controller.connect():
                    self.get_logger().info(f"Connected to serial port {self.serial_port}")
                    self._is_healthy = True
                else:
                    self.get_logger().error(f"Failed to connect to serial port {self.serial_port}")
                    self._is_healthy = False
            else:
                self.get_logger().warn("Running in dryrun mode - no actual serial communication")
                self._is_healthy = True

        except Exception as e:
            self.get_logger().error(f"Failed to initialize controller: {e}")
            self.controller = None
            self._is_healthy = False

    def _init_publishers(self):
       
        # 当前角度发布
        self.current_angle_pub = self.create_publisher(
            Float32MultiArray, '/gimbal/current_angle', 10
        )

        # 关节状态发布 (兼容JointState格式)
        self.joint_state_pub = self.create_publisher(
            JointState, '/gimbal/joint_states', 10
        )

        # 健康状态发布
        self.health_pub = self.create_publisher(
            Bool, '/gimbal/health', 10
        )

        # TF广播
        self.tf_broadcaster = TransformBroadcaster(self)

        # 调试信息发布
        self.debug_pub = self.create_publisher(
            String, '/gimbal/debug', 10
        )

    def _init_subscribers(self):
       
        # 目标角度订阅 (同步控制)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.target_angle_sub = self.create_subscription(
            Float32MultiArray,
            '/gimbal/target_angle',
            self._target_angle_callback,
            qos
        )

        # 分别控制pan/tilt的订阅
        self.pan_target_sub = self.create_subscription(
            Float32MultiArray,
            '/gimbal/pan_target',
            self._pan_target_callback,
            10
        )

        self.tilt_target_sub = self.create_subscription(
            Float32MultiArray,
            '/gimbal/tilt_target',
            self._tilt_target_callback,
            10
        )

        # 同步控制模式: 同时设置pan和tilt
        self.sync_target_sub = self.create_subscription(
            Float32MultiArray,
            '/gimbal/sync_target',
            self._sync_target_callback,
            10
        )

        # 特殊指令订阅
        self.command_sub = self.create_subscription(
            String,
            '/gimbal/command',
            self._command_callback,
            10
        )

    def _init_timers(self):
       
        # 状态更新定时器
        update_period = 1.0 / self.update_rate
        self.update_timer = self.create_timer(update_period, self._update_timer_callback)

        # 角度读取定时器
        read_period = 1.0 / self.read_rate
        self.read_timer = self.create_timer(read_period, self._read_timer_callback)

    # ========== 回调函数 ==========

    def _target_angle_callback(self, msg: Float32MultiArray):
       
        if len(msg.data) >= 2:
            pan_deg = float(msg.data[0])
            tilt_deg = float(msg.data[1])
            self._set_target_angle(pan_deg, tilt_deg)

    def _pan_target_callback(self, msg: Float32MultiArray):
        
        if len(msg.data) >= 1:
            self._set_pan_target(float(msg.data[0]))

    def _tilt_target_callback(self, msg: Float32MultiArray):
      
        if len(msg.data) >= 1:
            self._set_tilt_target(float(msg.data[0]))

    def _sync_target_callback(self, msg: Float32MultiArray):
      
        if len(msg.data) >= 2:
            pan_deg = float(msg.data[0])
            tilt_deg = float(msg.data[1])
            time_ms = int(msg.data[2]) if len(msg.data) >= 3 else None
            self._set_target_angle(pan_deg, tilt_deg, time_ms)

    def _command_callback(self, msg: String):
       
        command = msg.data.lower()
        self.get_logger().debug(f"Received command: {command}")

        if command == "stop":
            self._stop_all()
        elif command == "damping":
            self._enable_damping()
        elif command == "home":
            self._go_home()
        elif command == "calibrate":
            self._calibrate()
        elif command == "reset":
            self._reset_turns()

    # ========== 控制方法 ==========

    def _set_pan_target(self, degrees: float):
       
        self._target_pan = np.clip(degrees, self.pan_range[0], self.pan_range[1])
        if self.controller and not self.use_dryrun:
            self.controller.set_pan_angle(self._target_pan)

    def _set_tilt_target(self, degrees: float):
        
        self._target_tilt = np.clip(degrees, self.tilt_range[0], self.tilt_range[1])
        if self.controller and not self.use_dryrun:
            self.controller.set_tilt_angle(self._target_tilt)

    def _set_target_angle(self, pan_deg: float, tilt_deg: float, time_ms: int = None):
      
        pan_deg = np.clip(pan_deg, self.pan_range[0], self.pan_range[1])
        tilt_deg = np.clip(tilt_deg, self.tilt_range[0], self.tilt_range[1])

        self._target_pan = pan_deg
        self._target_tilt = tilt_deg

        if self.controller and not self.use_dryrun:
            self.controller.set_gimbal_angle(
                pan_deg, tilt_deg,
                time_ms=time_ms,
                sync=self.use_sync_control
            )

        self.get_logger().debug(f"Target: pan={pan_deg:.1f}, tilt={tilt_deg:.1f}")

    def _stop_all(self):
       
        if self.controller and not self.use_dryrun:
            self.controller.stop_all()

    def _enable_damping(self):
       
        if self.controller and not self.use_dryrun:
            self.controller.damping_all()

    def _go_home(self):
       
        self._set_target_angle(0.0, 0.0)

    def _calibrate(self):
       
        if self.controller and not self.use_dryrun:
            self.controller.set_origin(self.pan_servo_id)
            self.controller.set_origin(self.tilt_servo_id)

    def _reset_turns(self):
       
        if self.controller and not self.use_dryrun:
            self.controller.reset_turns(self.pan_servo_id)
            self.controller.reset_turns(self.tilt_servo_id)

    # ========== 定时器回调 ==========

    def _read_timer_callback(self):
       
        if not self.controller or self.use_dryrun:
            return

        try:
            pan_angle = self.controller.get_pan_angle()
            tilt_angle = self.controller.get_tilt_angle()

            if pan_angle is not None:
                self._current_pan = pan_angle
            if tilt_angle is not None:
                self._current_tilt = tilt_angle

        except Exception as e:
            self.get_logger().warn(f"Failed to read angles: {e}")
            self._is_healthy = False

    def _update_timer_callback(self):
        
        self._publish_current_angle()
        self._publish_joint_states()
        self._publish_health()
        self._publish_tf()

    # ========== 发布方法 ==========

    def _publish_current_angle(self):
       
        msg = Float32MultiArray()
        msg.data = [float(self._current_pan), float(self._current_tilt)]
        self.current_angle_pub.publish(msg)

    def _publish_joint_states(self):
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['pan_joint', 'tilt_joint']
        msg.position = [np.deg2rad(self._current_pan), np.deg2rad(self._current_tilt)]
        msg.velocity = [0.0, 0.0]  # 舵机不直接提供速度
        msg.effort = [0.0, 0.0]
        self.joint_state_pub.publish(msg)

    def _publish_health(self):
       
        msg = Bool()
        msg.data = self._is_healthy
        self.health_pub.publish(msg)

    def _publish_tf(self):
       
        now = self.get_clock().now().to_msg()

        # Pan joint transform
        pan_tf = TransformStamped()
        pan_tf.header.stamp = now
        pan_tf.header.frame_id = 'base_link'
        pan_tf.child_frame_id = 'pan_link'
        pan_tf.transform.translation.x = 0.0
        pan_tf.transform.translation.y = 0.0
        pan_tf.transform.translation.z = 0.0
        # 绕Z轴旋转
        yaw = np.deg2rad(self._current_pan)
        pan_tf.transform.rotation.w = np.cos(yaw / 2)
        pan_tf.transform.rotation.z = np.sin(yaw / 2)
        pan_tf.transform.rotation.x = 0.0
        pan_tf.transform.rotation.y = 0.0

        # Tilt joint transform
        tilt_tf = TransformStamped()
        tilt_tf.header.stamp = now
        tilt_tf.header.frame_id = 'pan_link'
        tilt_tf.child_frame_id = 'tilt_link'
        tilt_tf.transform.translation.x = 0.0
        tilt_tf.transform.translation.y = 0.0
        tilt_tf.transform.translation.z = 0.05  # 云台高度偏移
        # 绕Y轴旋转
        pitch = np.deg2rad(self._current_tilt)
        tilt_tf.transform.rotation.w = np.cos(pitch / 2)
        tilt_tf.transform.rotation.y = np.sin(pitch / 2)
        tilt_tf.transform.rotation.x = 0.0
        tilt_tf.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform([pan_tf, tilt_tf])

    def _publish_debug(self, message: str):
        
        msg = String()
        msg.data = message
        self.debug_pub.publish(msg)

    # ========== 生命周期 ==========

    def shutdown(self):
       
        self.get_logger().info("Shutting down GimbalNode...")
        if self.controller and not self.use_dryrun:
            self.controller.stop_all()
            self.controller.disconnect()
        self.get_logger().info("GimbalNode shutdown complete")


def main(args=None):
   
    rclpy.init(args=args)

    node = GimbalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
