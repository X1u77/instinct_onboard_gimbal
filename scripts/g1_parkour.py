import queue
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.base import ColdStartAgent
from instinct_onboard.agents.parkour_agent import ParkourAgent, ParkourStandAgent
from instinct_onboard.ros_nodes.realsense import UnitreeRsCameraNode

MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500


class G1ParkourNode(UnitreeRsCameraNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.available_agents = {}
        self.current_agent_name: str | None = None

    def register_agent(self, name: str, agent):
        self.available_agents[name] = agent

    def start_ros_handlers(self):
        super().start_ros_handlers()
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.main_loop_timer = self.create_timer(0.02, self.main_loop_callback)
        self.main_loop_timer_counter = 0
        self.main_loop_timer_counter_time = time.time()
        self.main_loop_callback_time_consumptions = queue.Queue(maxsize=MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)
        self.get_logger().info("Starting 31-DoF parkour loop at 50 Hz.")

    def main_loop_callback(self):
        start_time = time.time()
        if self.current_agent_name is None:
            self.get_logger().info("Starting cold start agent automatically.")
            self.current_agent_name = "cold_start"
            self.available_agents[self.current_agent_name].reset()
            return

        if self.current_agent_name == "cold_start":
            action, done = self.available_agents["cold_start"].step()
            self._send_agent_action("cold_start", action)
            if done:
                self.get_logger().info("Cold start done. Press L1 for parkour, R1 for stand.", once=True)
                if self.joy_stick_data.L1:
                    self._switch_agent("parkour")
                elif self.joy_stick_data.R1 and "stand" in self.available_agents:
                    self._switch_agent("stand")

        elif self.current_agent_name == "stand":
            action, _ = self.available_agents["stand"].step()
            self._send_agent_action("stand", action)
            if self.joy_stick_data.L1:
                self._switch_agent("parkour")

        elif self.current_agent_name == "parkour":
            action, _ = self.available_agents["parkour"].step()
            self._send_agent_action("parkour", action)
            if self.joy_stick_data.R1 and "stand" in self.available_agents:
                self._switch_agent("stand")

        self._log_main_loop_frequency(start_time)

    def _send_agent_action(self, agent_name: str, action: np.ndarray):
        agent = self.available_agents[agent_name]
        self.send_action(action, agent.action_offset, agent.action_scale, agent.p_gains, agent.d_gains)

    def _switch_agent(self, agent_name: str):
        self.get_logger().info(f"Switching to {agent_name} agent.")
        self.current_agent_name = agent_name
        self.available_agents[agent_name].reset()

    def _log_main_loop_frequency(self, start_time: float):
        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL <= 1:
            return
        self.main_loop_callback_time_consumptions.put(time.time() - start_time)
        self.main_loop_timer_counter += 1
        if self.main_loop_timer_counter % MAIN_LOOP_FREQUENCY_CHECK_INTERVAL != 0:
            return
        time_consumptions = [
            self.main_loop_callback_time_consumptions.get() for _ in range(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)
        ]
        self.get_logger().info(
            f"Actual main loop frequency: "
            f"{MAIN_LOOP_FREQUENCY_CHECK_INTERVAL / (time.time() - self.main_loop_timer_counter_time):.2f} Hz. "
            f"Mean callback time: {np.mean(time_consumptions):.4f} s."
        )
        self.main_loop_timer_counter = 0
        self.main_loop_timer_counter_time = time.time()


def main(args):
    rclpy.init()

    node = G1ParkourNode(
        rs_resolution=(480, 270),
        rs_fps=60,
        rs_serial_number=args.camera_serial,
        camera_individual_process=True,
        joint_pos_protect_ratio=2.0,
        robot_class_name="G1_31Dof_TorsoBase",
        dryrun=not args.nodryrun,
        enable_gimbal=args.gimbal,
        gimbal_serial_port=args.gimbal_port,
        gimbal_pan_servo_id=args.gimbal_pan_id,
        gimbal_tilt_servo_id=args.gimbal_tilt_id,
        gimbal_pan_range=tuple(np.rad2deg([-1.6, 1.6])),
        gimbal_tilt_range=tuple(np.rad2deg([0.5, 1.5])),
    )

    parkour_agent = ParkourAgent(
        logdir=args.logdir,
        ros_node=node,
        depth_vis=args.depth_vis,
        pointcloud_vis=args.pointcloud_vis,
        lin_vel_deadband=args.lin_vel_deadband,
        lin_vel_range=args.lin_vel_range,
        ang_vel_deadband=args.ang_vel_deadband,
        ang_vel_range=args.ang_vel_range,
        debug_policy_io=args.debug_policy_io,
    )
    node.register_agent("parkour", parkour_agent)

    if args.standdir is not None:
        stand_agent = ParkourStandAgent(logdir=args.standdir, ros_node=node)
        node.register_agent("stand", stand_agent)

    cold_start_agent = ColdStartAgent(
        startup_step_size=args.startup_step_size,
        ros_node=node,
        joint_target_pos=parkour_agent.default_joint_pos,
        action_scale=parkour_agent.action_scale,
        action_offset=parkour_agent.action_offset,
        p_gains=parkour_agent.p_gains * args.kpkd_factor,
        d_gains=parkour_agent.d_gains * args.kpkd_factor,
    )
    node.register_agent("cold_start", cold_start_agent)

    if args.depth_vis or args.pointcloud_vis:
        node.publish_auxiliary_static_transforms("realsense_depth_link_transform")

    node.start_ros_handlers()
    node.get_logger().info("G1ParkourNode is ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    import argparse

    def _parse_2float(s):
        values = s.split() if isinstance(s, str) else s
        if len(values) != 2:
            raise argparse.ArgumentTypeError(f"Expected 2 floats, got {values!r}")
        return [float(values[0]), float(values[1])]

    parser = argparse.ArgumentParser(description="G1 31-DoF parkour deployment node")
    parser.add_argument("--logdir", type=str, required=True, help="Directory containing actor/depth ONNX and YAML files")
    parser.add_argument("--standdir", type=str, default=None, help="Optional stand policy directory")
    parser.add_argument("--camera_serial", type=str, default="420122071680")
    parser.add_argument("--startup_step_size", type=float, default=0.2)
    parser.add_argument("--kpkd_factor", type=float, default=2.0)
    parser.add_argument("--depth_vis", action="store_true", default=False)
    parser.add_argument("--pointcloud_vis", action="store_true", default=False)
    parser.add_argument("--lin_vel_deadband", type=float, default=0.5)
    parser.add_argument("--lin_vel_range", type=_parse_2float, default=[0.5, 0.5])
    parser.add_argument("--ang_vel_deadband", type=float, default=0.15)
    parser.add_argument("--ang_vel_range", type=_parse_2float, default=[0.0, 1.0])
    parser.add_argument("--nodryrun", action="store_true", default=False)
    parser.add_argument("--gimbal", action="store_true", default=False, help="Enable UART head gimbal control")
    parser.add_argument("--gimbal_port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--gimbal_pan_id", type=int, default=0)
    parser.add_argument("--gimbal_tilt_id", type=int, default=1)
    parser.add_argument("--debug_policy_io", action="store_true", default=False)
    parser.add_argument("--debug", action="store_true", default=False)

    args = parser.parse_args()
    if args.debug:
        import debugpy

        ip_address = ("0.0.0.0", 6789)
        print("Process: " + " ".join(sys.argv[:]))
        print("Is waiting for attach at address: %s:%d" % ip_address, flush=True)
        debugpy.listen(ip_address)
        debugpy.wait_for_client()
        debugpy.breakpoint()

    main(args)
