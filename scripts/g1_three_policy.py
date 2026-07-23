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

from instinct_onboard.agents.base import ColdStartAgent
from instinct_onboard.agents.parkour_agent import Body29DepthOn31Agent, ParkourAgent
from instinct_onboard.agents.walk_agent import Body29ActorOn31Agent
from instinct_onboard.ros_nodes.realsense import UnitreeRsCameraNode

MAIN_LOOP_FREQUENCY_CHECK_INTERVAL = 500


class G1ThreePolicyNode(UnitreeRsCameraNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.available_agents = dict()
        self.current_agent_name: str | None = None
        self.current_speed_scale = 0.0
        self.parkour_blend_duration = 0.0
        self._transition_start_time = None
        self._transition_start_joint_pos = None

    def register_agent(self, name: str, agent):
        self.available_agents[name] = agent

    def start_ros_handlers(self):
        super().start_ros_handlers()
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        main_loop_duration = 0.02
        self.get_logger().info(f"Starting main loop with duration: {main_loop_duration} seconds.")
        self.main_loop_timer = self.create_timer(main_loop_duration, self.main_loop_callback)
        if MAIN_LOOP_FREQUENCY_CHECK_INTERVAL > 1:
            self.main_loop_timer_counter = 0
            self.main_loop_timer_counter_time = time.time()
            self.main_loop_callback_time_consumptions = queue.Queue(maxsize=MAIN_LOOP_FREQUENCY_CHECK_INTERVAL)

    def _switch_to(self, agent_name: str, reason: str):
        if self.current_agent_name == agent_name:
            return
        self.get_logger().info(reason)
        self.current_agent_name = agent_name
        self.available_agents[self.current_agent_name].reset()
        if agent_name == "parkour" and self.parkour_blend_duration > 0.0:
            self._transition_start_time = time.monotonic()
            self._transition_start_joint_pos = self.joint_pos_.copy()
        else:
            self._transition_start_time = None
            self._transition_start_joint_pos = None

    def _set_speed_scale(self, speed_scale: float, reason=None):
        speed_scale = float(speed_scale)
        self.current_speed_scale = speed_scale

        parkour_agent = self.available_agents["parkour"]
        if hasattr(parkour_agent, "set_speed_scale"):
            parkour_agent.set_speed_scale(speed_scale)

        if reason:
            self.get_logger().info(reason)

    def _step_current_agent(self):
        if not self.dryrun and not self.gimbal_feedback_is_fresh():
            self.get_logger().error("Gimbal feedback is missing or stale; shutting down motor commands.")
            self._turn_off_motors()
            raise SystemExit()
        agent = self.available_agents[self.current_agent_name]
        action, done = agent.step()
        if self.current_agent_name == "parkour" and not self.dryrun and not self.rs_data_is_fresh():
            self.get_logger().error("D455 depth frames are missing or stale; shutting down motor commands.")
            self._turn_off_motors()
            raise SystemExit()
        if self.current_agent_name == "parkour" and self._transition_start_time is not None:
            elapsed = time.monotonic() - self._transition_start_time
            alpha = float(np.clip(elapsed / self.parkour_blend_duration, 0.0, 1.0))
            policy_target = action * agent.action_scale + agent.action_offset
            blended_target = (1.0 - alpha) * self._transition_start_joint_pos + alpha * policy_target
            blended_action = np.zeros_like(action)
            scale_nonzero = np.abs(agent.action_scale) > 1e-8
            np.divide(
                blended_target - agent.action_offset,
                agent.action_scale,
                out=blended_action,
                where=scale_nonzero,
            )
            action = blended_action
            if alpha >= 1.0:
                self._transition_start_time = None
                self._transition_start_joint_pos = None
                self.get_logger().info("Parkour smooth takeover complete.")
        self.send_action(
            action,
            agent.action_offset,
            agent.action_scale,
            agent.p_gains,
            agent.d_gains,
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

        self._publish_joint_states()

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

    def _publish_joint_states(self):
        """Publish body and measured revised-head joints for robot_state_publisher/RViz."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.sim_joint_names)
        msg.position = self.joint_pos_.astype(float).tolist()
        msg.velocity = self.joint_vel_.astype(float).tolist()
        self.joint_state_publisher.publish(msg)


def main(args):
    rclpy.init()

    node = G1ThreePolicyNode(
        rs_resolution=(args.camera_width, args.camera_height),
        rs_fps=args.camera_fps,
        rs_vfov_deg=57.0,
        rs_serial_number=args.camera_serial,
        camera_individual_process=True,
        joint_pos_protect_ratio=2.0,
        robot_class_name="G1_31Dof_D455",
        dryrun=not args.nodryrun,
        enable_gimbal=args.gimbal,
        gimbal_backend="g1_comp_service",
        gimbal_pan_range=(-50.0, 50.0),
        gimbal_tilt_range=(-20.0, 85.0),
    )
    if args.nodryrun and node.gimbal is None:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("Gimbal connection failed; refusing to start the 31-DoF deployment.")

    parkour_kwargs = dict(
        logdir=args.logdir,
        ros_node=node,
        depth_vis=args.depth_vis,
        pointcloud_vis=args.pointcloud_vis,
    )

    parkour_signature = inspect.signature(ParkourAgent)
    if "initial_speed_scale" in parkour_signature.parameters:
        parkour_kwargs["initial_speed_scale"] = 0.0
    if "debug_policy_io" in parkour_signature.parameters:
        parkour_kwargs["debug_policy_io"] = args.debug_policy_io
    if "freeze_head" in parkour_signature.parameters:
        parkour_kwargs["freeze_head"] = args.parkour_freeze_head

    parkour_agent = ParkourAgent(**parkour_kwargs)
    node.parkour_blend_duration = max(0.0, args.parkour_blend_duration)
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

    parser = argparse.ArgumentParser(description="G1 deployment node with 29dof stand, 29dof walk and 31dof parkour")
    parser.add_argument("--stand_logdir", type=str, required=True, help="Directory for the 29dof stand agent")
    parser.add_argument("--walk_logdir", type=str, required=True, help="Directory for the 29dof walk agent")
    parser.add_argument("--logdir", type=str, required=True, help="Directory for the revised-head 31dof parkour agent")
    parser.add_argument(
        "--camera_serial",
        type=str,
        default=None,
        help="D455 serial number. If omitted, use the first connected RealSense.",
    )
    parser.add_argument("--camera_width", type=int, default=848, help="D455 depth width (default: 848)")
    parser.add_argument("--camera_height", type=int, default=480, help="D455 depth height (default: 480)")
    parser.add_argument("--camera_fps", type=int, default=60, help="D455 depth FPS (default: 60)")
    parser.add_argument(
        "--parkour_freeze_head",
        action="store_true",
        default=False,
        help="Keep parkour head yaw/pitch at the training default pose",
    )
    parser.add_argument(
        "--parkour_blend_duration",
        type=float,
        default=0.5,
        help="Seconds to blend from the current pose into parkour targets (default: 0.5)",
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
        "--debug",
        action="store_true",
        default=False,
        help="Enable debug mode (default: False)",
    )
    parser.add_argument(
        "--debug_policy_io",
        action="store_true",
        default=False,
        help="Log 31dof parkour raw action and scaled joint targets for debugging (default: False)",
    )

    args = parser.parse_args()
    if args.nodryrun and not args.gimbal:
        parser.error(
            "--nodryrun requires --gimbal because the 31dof parkour policy controls head pitch "
            "and consumes measured head feedback."
        )
    if args.debug:
        import debugpy

        ip_address = ("0.0.0.0", 6789)
        print("Process: " + " ".join(sys.argv[:]))
        print("Is waiting for attach at address: %s:%d" % ip_address, flush=True)
        debugpy.listen(ip_address)
        debugpy.wait_for_client()
        debugpy.breakpoint()

    main(args)
