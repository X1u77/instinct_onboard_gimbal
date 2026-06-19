import queue
import sys
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import time
import inspect
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from instinct_onboard.agents.base import ColdStartAgent
from instinct_onboard.agents.parkour_agent import Body29DepthOn31Agent, ParkourAgent
from instinct_onboard.agents.walk_agent import Body29ActorOn31Agent
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

    def _set_speed_scale(self, speed_scale: float, reason=None):
        speed_scale = float(speed_scale)
        self.current_speed_scale = speed_scale

        parkour_agent = self.available_agents["parkour"]
        if hasattr(parkour_agent, "set_speed_scale"):
            parkour_agent.set_speed_scale(speed_scale)

        if reason:
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
                self.get_logger().info(
                    "ColdStartAgent done. Press 'R1' for 29dof stand, 'A' for 29dof walk, 'L1' for 31dof parkour.",
                    throttle_duration_sec=10.0,
                )
            if done and self.joy_stick_data.R1:
                self._switch_to("stand", "R1 button pressed, switching to 29dof stand.")
            elif done and self.joy_stick_data.A:
                self._switch_to("walk", "A button pressed, switching to 29dof walk.")
            elif done and self.joy_stick_data.L1:
                self._set_speed_scale(0.0, "L1 button pressed, resetting parkour speed_scale=0.0.")
                self._switch_to("parkour", "L1 button pressed, switching to 31dof parkour.")

        elif self.current_agent_name == "stand":
            self._step_current_agent()
            if self.joy_stick_data.R1:
                self._switch_to("stand", "R1 button pressed, switching to 29dof stand.")
            elif self.joy_stick_data.A:
                self._switch_to("walk", "A button pressed, switching to 29dof walk.")
            elif self.joy_stick_data.L1:
                self._set_speed_scale(0.0, "L1 button pressed, resetting parkour speed_scale=0.0.")
                self._switch_to("parkour", "L1 button pressed, switching to 31dof parkour.")

        elif self.current_agent_name == "walk":
            self._step_current_agent()
            if self.joy_stick_data.R1:
                self._switch_to("stand", "R1 button pressed, switching to 29dof stand.")
            elif self.joy_stick_data.A:
                self._switch_to("walk", "A button pressed, switching to 29dof walk.")
            elif self.joy_stick_data.L1:
                self._set_speed_scale(0.0, "L1 button pressed, resetting parkour speed_scale=0.0.")
                self._switch_to("parkour", "L1 button pressed, switching to 31dof parkour.")

        elif self.current_agent_name == "parkour":
            self._step_current_agent()
            if self.joy_stick_data.R1:
                self._switch_to("stand", "R1 button pressed, switching to 29dof stand.")
            elif self.joy_stick_data.A:
                self._switch_to("walk", "A button pressed, switching to 29dof walk.")

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
        gimbal_tilt_range=(float(np.rad2deg(0.5)), 60.0),
    )

    publish_depth_image = args.depth_vis or args.publish_depth_image or args.debug_windows
    print_policy_io = args.debug_policy_io or args.debug_windows
    parkour_kwargs = dict(
        logdir=args.logdir,
        ros_node=node,
        depth_vis=publish_depth_image,
        pointcloud_vis=args.pointcloud_vis,
    )

    parkour_signature = inspect.signature(ParkourAgent)
    if "initial_speed_scale" in parkour_signature.parameters:
        parkour_kwargs["initial_speed_scale"] = 0.0
    if "debug_policy_io" in parkour_signature.parameters:
        parkour_kwargs["debug_policy_io"] = args.debug_policy_io

    parkour_agent = ParkourAgent(**parkour_kwargs)
    if hasattr(parkour_agent, "set_speed_scale"):
        parkour_agent.set_speed_scale(0.0)

    stand_agent = Body29DepthOn31Agent(
        logdir=args.stand_logdir,
        ros_node=node,
    )
    walk_agent = Body29ActorOn31Agent(
        logdir=args.walk_logdir,
        ros_node=node,
    )

    for agent_name, agent in {
        "stand": stand_agent,
        "walk": walk_agent,
        "parkour": parkour_agent,
    }.items():
        if hasattr(agent, "configure_policy_io_debug"):
            agent.configure_policy_io_debug(
                agent_name=agent_name,
                print_enabled=print_policy_io,
                logdir=args.policy_io_logdir,
                interval=args.policy_io_interval,
            )

    if hasattr(node, "configure_debug_windows"):
        node.configure_debug_windows(
            enabled=args.debug_windows,
            show_depth=publish_depth_image,
            show_policy_io=print_policy_io,
        )
    joint_tracking_logdir = args.joint_tracking_logdir
    if joint_tracking_logdir is None and args.policy_io_logdir:
        joint_tracking_logdir = os.path.join(args.policy_io_logdir, "joint_tracking")
    record_joint_tracking = args.record_joint_tracking or joint_tracking_logdir is not None
    if hasattr(node, "configure_joint_tracking"):
        node.configure_joint_tracking(
            enabled=record_joint_tracking,
            logdir=joint_tracking_logdir,
            interval=args.joint_tracking_interval,
            show_plot=args.show_joint_tracking_plot,
        )

    node.register_agent("stand", stand_agent)
    node.register_agent("walk", walk_agent)
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

    if publish_depth_image or args.pointcloud_vis:
        node.publish_auxiliary_static_transforms("realsense_depth_link_transform")
    if publish_depth_image:
        node.get_logger().info("Depth image debug stream is published on /debug/depth_image.")
    if args.policy_io_logdir:
        node.get_logger().info(f"Policy I/O tensors are recorded under {args.policy_io_logdir}.")
    if args.debug_windows:
        node.get_logger().info("Debug windows are enabled for depth image and policy I/O summaries.")
    if record_joint_tracking:
        node.get_logger().info(f"Joint tracking will be recorded under {joint_tracking_logdir}.")

    node.start_ros_handlers()
    node.get_logger().info("G1ThreePolicyNode is ready to run.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    finally:
        if hasattr(node, "finalize_joint_tracking"):
            node.finalize_joint_tracking()
        if hasattr(node, "close_debug_windows"):
            node.close_debug_windows()
        node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="G1 deployment node with 29dof stand, 29dof walk and 31dof parkour")
    parser.add_argument("--stand_logdir", type=str, help="Directory to load the 29dof stand agent from")
    parser.add_argument("--walk_logdir", type=str, help="Directory to load the 29dof walk agent from")
    parser.add_argument("--logdir", type=str, help="Directory to load the 31dof parkour agent from")
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
        "--publish_depth_image",
        action="store_true",
        default=False,
        help="Publish the policy depth image on /debug/depth_image (alias for depth image debug stream)",
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
    parser.add_argument(
        "--debug_policy_io",
        action="store_true",
        default=False,
        help="Print compact policy input/output tensor stats in the terminal (default: False)",
    )
    parser.add_argument(
        "--policy_io_logdir",
        type=str,
        default=None,
        help="Directory to record policy input/output summaries and full .npz tensor samples",
    )
    parser.add_argument(
        "--policy_io_interval",
        type=float,
        default=0.5,
        help="Minimum seconds between policy I/O debug records per agent (default: 0.5)",
    )
    parser.add_argument(
        "--debug_windows",
        action="store_true",
        default=False,
        help="Show live OpenCV windows for the policy depth image and policy I/O summaries",
    )
    parser.add_argument(
        "--record_joint_tracking",
        action="store_true",
        default=False,
        help="Record expected, commanded and actual joint positions for plotting after shutdown",
    )
    parser.add_argument(
        "--joint_tracking_logdir",
        type=str,
        default=None,
        help="Directory to save joint_tracking.npz and joint_tracking.png; defaults under policy_io_logdir when set",
    )
    parser.add_argument(
        "--joint_tracking_interval",
        type=float,
        default=0.02,
        help="Minimum seconds between joint tracking samples (default: 0.02)",
    )
    parser.add_argument(
        "--show_joint_tracking_plot",
        action="store_true",
        default=False,
        help="Show the joint tracking matplotlib figure after shutdown in addition to saving it",
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
