import queue
import sys
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.base import ColdStartAgent
from instinct_onboard.agents.parkour_agent import Body29DepthOn31Agent, ParkourAgent
from instinct_onboard.ros_nodes.realsense import UnitreeRsCameraNode

MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500
DEFAULT_CAMERA_SERIAL = "420122071680"


class G1ThreePolicyNode(UnitreeRsCameraNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.available_agents = dict()
        self.current_agent_name: str | None = None
        self.current_speed_scale = 0.0

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

    def _set_speed_scale(self, speed_scale: float, reason: str):
        speed_scale = float(speed_scale)
        if abs(self.current_speed_scale - speed_scale) < 1e-6:
            return
        self.current_speed_scale = speed_scale
        self.available_agents["parkour"].set_speed_scale(speed_scale)
        self.get_logger().info(reason)

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
                self.current_agent_name = "stand"
                self.available_agents[self.current_agent_name].reset()
                self.get_logger().info(
                    "ColdStartAgent done. Entering 29dof stand policy. Press 'R1' for stand, 'L1' for 31dof parkour."
                )

        elif self.current_agent_name == "stand":
            self._step_current_agent()
            if self.joy_stick_data.R1:
                self._switch_to("stand", "R1 button pressed, switching to 29dof stand.")
            elif self.joy_stick_data.L1:
                self._set_speed_scale(0.5, "L1 button pressed, setting speed_scale=0.5.")
                self._switch_to("parkour", "L1 button pressed, switching to 31dof parkour.")

        elif self.current_agent_name == "parkour":
            self._step_current_agent()
            if self.joy_stick_data.R1:
                self._switch_to("stand", "R1 button pressed, switching to 29dof stand.")
            elif self.joy_stick_data.L1:
                self._set_speed_scale(0.5, "L1 button pressed, setting speed_scale=0.5.")

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
        rs_serial_number=DEFAULT_CAMERA_SERIAL,
        camera_individual_process=True,
        joint_pos_protect_ratio=2.0,
        robot_class_name="G1_31Dof_TorsoBase",
        dryrun=not args.nodryrun,
        enable_gimbal=args.gimbal,
        gimbal_serial_port=args.gimbal_port,
        gimbal_pan_range=tuple(np.rad2deg([-1.6, 1.6])),
        gimbal_tilt_range=tuple(np.rad2deg([0.5, 1.5])),
    )

    parkour_agent = ParkourAgent(
        logdir=args.logdir,
        ros_node=node,
        depth_vis=args.depth_vis,
        pointcloud_vis=args.pointcloud_vis,
        initial_speed_scale=0.0,
    )
    parkour_agent.set_speed_scale(0.0)

    stand_agent = Body29DepthOn31Agent(
        logdir=args.stand_logdir,
        ros_node=node,
    )

    node.register_agent("stand", stand_agent)
    node.register_agent("parkour", parkour_agent)

    cold_start_agent = ColdStartAgent(
        startup_step_size=args.startup_step_size,
        ros_node=node,
        joint_target_pos=stand_agent.default_joint_pos,
        action_scale=stand_agent.action_scale,
        action_offset=stand_agent.action_offset,
        p_gains=stand_agent.p_gains * args.kpkd_factor,
        d_gains=stand_agent.d_gains * args.kpkd_factor,
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

    parser = argparse.ArgumentParser(description="G1 deployment node with 29dof stand and 31dof parkour")
    parser.add_argument("--logdir", type=str, help="Directory to load the 31dof parkour agent from")
    parser.add_argument("--stand_logdir", type=str, help="Directory to load the 29dof stand agent from")
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
