import queue
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.base import ColdStartAgent
from instinct_onboard.agents.parkour_agent import ParkourAgent
from instinct_onboard.agents.walk_agent import Body29ActorOn31Agent
from instinct_onboard.ros_nodes.realsense import UnitreeRsCameraNode

MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500


class G1ThreePolicyNode(UnitreeRsCameraNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.available_agents = dict()
        self.current_agent_name: str | None = None

    def register_agent(self, name: str, agent):
        self.available_agents[name] = agent

    def start_ros_handlers(self):
        super().start_ros_handlers()
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        main_loop_duration = 0.02
        self.get_logger().info(f"Starting main loop with duration: {main_loop_duration} seconds.")
        self.main_loop_timer = self.create_timer(main_loop_duration, self.main_loop_callback)
        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_timer_counter = 0
            self.main_loop_timer_counter_time = time.time()
            self.main_loop_callback_time_consumptions = queue.Queue(maxsize=MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)

    def _switch_to(self, agent_name: str, reason: str):
        self.get_logger().info(reason)
        self.current_agent_name = agent_name
        self.available_agents[self.current_agent_name].reset()

    def _step_current_agent(self):
        action, done = self.available_agents[self.current_agent_name].step()
        self.send_action(
            action,
            self.available_agents[self.current_agent_name].action_offset,
            self.available_agents[self.current_agent_name].action_scale,
            self.available_agents[self.current_agent_name].p_gains,
            self.available_agents[self.current_agent_name].d_gains,
        )
        return done

    def main_loop_callback(self):
        start_time = time.time()
        if self.current_agent_name is None:
            self.get_logger().info("Starting cold start agent automatically.")
            self.current_agent_name = "cold_start"
            self.available_agents[self.current_agent_name].reset()
            return

        if self.current_agent_name == "cold_start":
            done = self._step_current_agent()
            if done:
                self.get_logger().info(
                    "ColdStartAgent done. Press 'L1' for 29dof stand, 'UP' for 29dof parkour, 'R1' for 31dof parkour."
                )
            if done and self.joy_stick_data.L1:
                self._switch_to("stand29", "L1 button pressed, switching to 29dof stand.")
            elif done and self.joy_stick_data.up:
                self._switch_to("parkour29", "UP button pressed, switching to 29dof parkour.")
            elif done and self.joy_stick_data.R1:
                self._switch_to("parkour31", "R1 button pressed, switching to 31dof parkour.")

        elif self.current_agent_name == "stand29":
            self._step_current_agent()
            if self.joy_stick_data.up:
                self._switch_to("parkour29", "UP button pressed, switching to 29dof parkour.")
            elif self.joy_stick_data.R1:
                self._switch_to("parkour31", "R1 button pressed, switching to 31dof parkour.")

        elif self.current_agent_name == "parkour29":
            self._step_current_agent()
            if self.joy_stick_data.L1:
                self._switch_to("stand29", "L1 button pressed, switching to 29dof stand.")
            elif self.joy_stick_data.R1:
                self._switch_to("parkour31", "R1 button pressed, switching to 31dof parkour.")

        elif self.current_agent_name == "parkour31":
            self._step_current_agent()
            if self.joy_stick_data.L1:
                self._switch_to("stand29", "L1 button pressed, switching to 29dof stand.")
            elif self.joy_stick_data.up:
                self._switch_to("parkour29", "UP button pressed, switching to 29dof parkour.")

        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_callback_time_consumptions.put(time.time() - start_time)
            self.main_loop_timer_counter += 1
            if self.main_loop_timer_counter % MAIN_LOOP_FREQUENCY_CHECK_INTERVAL == 0:
                time_consumptions = [
                    self.main_loop_callback_time_consumptions.get() for _ in range(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)
                ]
                self.get_logger().info(
                    f"Actual main loop frequency: {(MAIN_LOOP_FREQUENCY_CHECK_INTERVAL / (time.time() - self.main_loop_timer_counter_time)):.2f} Hz. Mean time consumption: {np.mean(time_consumptions):.4f} s."
                )
                self.main_loop_timer_counter = 0
                self.main_loop_timer_counter_time = time.time()


def main(args):
    rclpy.init()

    node = G1ThreePolicyNode(
        rs_resolution=(480, 270),
        rs_fps=60,
        rs_serial_number=args.camera_serial,
        camera_individual_process=True,
        joint_pos_protect_ratio=2.0,
        robot_class_name="G1_31Dof_TorsoBase",
        dryrun=not args.nodryrun,
        enable_gimbal=args.gimbal,
        gimbal_serial_port=args.gimbal_port,
        gimbal_pan_range=tuple(np.rad2deg([-1.6, 1.6])),
        gimbal_tilt_range=tuple(np.rad2deg([0.5, 1.5])),
    )

    stand29_agent = Body29ActorOn31Agent(
        logdir=args.stand29_logdir,
        ros_node=node,
    )
    parkour29_agent = Body29ActorOn31Agent(
        logdir=args.parkour29_logdir,
        ros_node=node,
    )
    parkour31_agent = ParkourAgent(
        logdir=args.parkour31_logdir,
        ros_node=node,
        depth_vis=args.depth_vis,
        pointcloud_vis=args.pointcloud_vis,
        lin_vel_deadband=args.lin_vel_deadband,
        lin_vel_range=args.lin_vel_range,
        ang_vel_deadband=args.ang_vel_deadband,
        ang_vel_range=args.ang_vel_range,
    )

    node.register_agent("stand29", stand29_agent)
    node.register_agent("parkour29", parkour29_agent)
    node.register_agent("parkour31", parkour31_agent)

    cold_start_agent = ColdStartAgent(
        startup_step_size=args.startup_step_size,
        ros_node=node,
        joint_target_pos=stand29_agent.default_joint_pos,
        action_scale=stand29_agent.action_scale,
        action_offset=stand29_agent.action_offset,
        p_gains=stand29_agent.p_gains * args.kpkd_factor,
        d_gains=stand29_agent.d_gains * args.kpkd_factor,
    )
    node.register_agent("cold_start", cold_start_agent)

    if args.depth_vis or args.pointcloud_vis:
        node.publish_auxiliary_static_transforms("realsense_depth_link_transform")

    node.start_ros_handlers()
    node.get_logger().info("G1ThreePolicyNode is ready to run.")
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
        values = s.split()
        if len(values) != 2:
            raise argparse.ArgumentTypeError(f"Expected 2 floats, got {len(values)}: {s!r}")
        try:
            return [float(values[0]), float(values[1])]
        except ValueError as e:
            raise argparse.ArgumentTypeError(f"Invalid float value: {e}")

    parser = argparse.ArgumentParser(description="G1 three-policy deployment node")
    parser.add_argument("--stand29_logdir", type=str, help="Directory to load the 29dof stand agent from")
    parser.add_argument("--parkour29_logdir", type=str, help="Directory to load the 29dof parkour agent from")
    parser.add_argument("--parkour31_logdir", type=str, help="Directory to load the 31dof parkour agent from")
    parser.add_argument(
        "--camera_serial",
        type=str,
        default="420122071680",
        help="Serial number of the RealSense device to use (default: 420122071680)",
    )
    parser.add_argument(
        "--startup_step_size",
        type=float,
        default=0.2,
        help="Startup step size for the cold start agent (default: 0.2)",
    )
    parser.add_argument(
        "--kpkd_factor",
        type=float,
        default=2.0,
        help="KPKD factor for the cold start agent (default: 2.0)",
    )
    parser.add_argument(
        "--depth_vis",
        action="store_true",
        default=False,
        help="Visualize the depth image for the 31dof parkour policy (default: False)",
    )
    parser.add_argument(
        "--pointcloud_vis",
        action="store_true",
        default=False,
        help="Visualize the pointcloud for the 31dof parkour policy (default: False)",
    )
    parser.add_argument(
        "--lin_vel_deadband",
        type=float,
        default=0.5,
        help="Deadband of wireless control for linear velocity (default: 0.5)",
    )
    parser.add_argument(
        "--lin_vel_range",
        type=_parse_2float,
        default=[0.5, 0.5],
        help="Range of linear velocity for the 31dof parkour policy (default: [0.5 0.5])",
    )
    parser.add_argument(
        "--ang_vel_deadband",
        type=float,
        default=0.5,
        help="Deadband of wireless control for angular velocity (default: 0.5)",
    )
    parser.add_argument(
        "--ang_vel_range",
        type=_parse_2float,
        default=[0.0, 1.0],
        help="Range of angular velocity for the 31dof parkour policy (default: [0.0 1.0])",
    )
    parser.add_argument(
        "--nodryrun",
        action="store_true",
        default=False,
        help="Run the node without dry run mode (default: False)",
    )
    parser.add_argument(
        "--gimbal",
        action="store_true",
        default=False,
        help="Enable gimbal control for head joints (default: False)",
    )
    parser.add_argument(
        "--gimbal_port",
        type=str,
        default="/dev/ttyUSB0",
        help="Serial port for gimbal servo (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        default=False,
        help="Enable debug mode (default: False)",
    )

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
