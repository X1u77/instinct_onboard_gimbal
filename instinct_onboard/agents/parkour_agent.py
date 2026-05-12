from __future__ import annotations

import time
import re

import cv2
import numpy as np
import onnxruntime as ort
import ros2_numpy as rnp
import yaml
from sensor_msgs.msg import Image, PointCloud2

from instinct_onboard.agents.base import OnboardAgent
from instinct_onboard.ros_nodes.base import RealNode
from instinct_onboard.utils import CircularBuffer


class ParkourAgent(OnboardAgent):
    """31-DoF parkour policy with a depth encoder and 2-DoF head gimbal actions."""

    rs_resolution = (480, 270)
    rs_frequency = 60

    def __init__(
        self,
        logdir: str,
        ros_node: RealNode,
        depth_vis: bool = False,
        pointcloud_vis: bool = False,
        initial_speed_scale: float = 0.0,
        lin_vel_deadband: float = 0.5,
        lin_vel_range: list[float] | tuple[float, float] = (0.5, 0.5),
        ang_vel_deadband: float = 0.15,
        ang_vel_range: list[float] | tuple[float, float] = (0.0, 1.0),
        debug_policy_io: bool = False,
    ):
        super().__init__(logdir, ros_node)
        self.ort_sessions: dict[str, ort.InferenceSession] = {}
        self.speed_scale = float(initial_speed_scale)
        self.lin_vel_deadband = lin_vel_deadband
        self.ang_vel_deadband = ang_vel_deadband
        self.lin_vel_range = lin_vel_range
        self.ang_vel_range = ang_vel_range
        self.debug_policy_io = debug_policy_io
        self._last_policy_io_log_time = 0.0
        self.xyyaw_command = np.array([self.speed_scale], dtype=np.float32)

        self._parse_obs_config()
        self._parse_action_config()
        self._load_models()

        self.debug_depth_publisher = (
            self.ros_node.create_publisher(Image, "/debug/depth_image", 10) if depth_vis else None
        )
        self.debug_pointcloud_publisher = (
            self.ros_node.create_publisher(PointCloud2, "/debug/pointcloud", 10) if pointcloud_vis else None
        )

    def _parse_obs_config(self):
        super()._parse_obs_config()
        with open(self._resolve_logdir_path("params", "agent.yaml")) as f:
            self.agent_cfg = yaml.unsafe_load(f)

        self.proprio_obs_names = [obs_name for obs_name in self.obs_funcs if "depth" not in obs_name]
        self.depth_obs_names = [obs_name for obs_name in self.obs_funcs if "depth" in obs_name]
        if len(self.depth_obs_names) != 1:
            raise ValueError(f"Expected one depth observation, got {self.depth_obs_names}")
        self._parse_depth_image_config()

    def _parse_observation_function(self, obs_name: str, obs_config: dict):
        obs_func = obs_config["func"].split(":")[-1]
        if obs_func == "delayed_visualizable_image":
            self.obs_funcs[obs_name] = self._get_delayed_visualizable_image_obs
            return
        if obs_func == "camera_offset_yaw_pitch":
            self.obs_funcs[obs_name] = self._get_camera_offset_yaw_pitch_obs
            return
        return super()._parse_observation_function(obs_name, obs_config)

    def _parse_action_config(self):
        super()._parse_action_config()
        self._camera_action_joint_ids: list[int] = []
        self._camera_action_ranges: list[tuple[float, float]] = []
        camera_cfg = self.cfg["actions"].get("camera_yaw_pitch")
        if camera_cfg is not None:
            self._camera_action_joint_ids = [
                self.ros_node.sim_joint_names.index(camera_cfg["yaw_joint_name"]),
                self.ros_node.sim_joint_names.index(camera_cfg["pitch_joint_name"]),
            ]
            self._camera_action_ranges = [
                tuple(camera_cfg.get("yaw_range", (-np.inf, np.inf))),
                tuple(camera_cfg.get("pitch_range", (-np.inf, np.inf))),
            ]

    def _parse_depth_image_config(self):
        camera_cfg = self.cfg["scene"]["camera"]
        self.output_resolution = (
            int(camera_cfg["pattern_cfg"]["width"]),
            int(camera_cfg["pattern_cfg"]["height"]),
        )
        noise_pipeline = camera_cfg["noise_pipeline"]
        depth_norm_cfg = noise_pipeline["depth_normalization"]
        self.depth_range = depth_norm_cfg["depth_range"]
        self.depth_output_range = (
            depth_norm_cfg["output_range"] if depth_norm_cfg["normalize"] else self.depth_range
        )
        self.crop_region = noise_pipeline.get("crop_and_resize", {}).get("crop_region", (0, 0, 0, 0))
        self.gaussian_kernel_size = None
        if "gaussian_blur" in noise_pipeline:
            kernel_size = int(noise_pipeline["gaussian_blur"]["kernel_size"])
            self.gaussian_kernel_size = (kernel_size, kernel_size)
            self.gaussian_sigma = noise_pipeline["gaussian_blur"]["sigma"]

        self.depth_width = self.output_resolution[0] - self.crop_region[2] - self.crop_region[3]
        self.depth_height = self.output_resolution[1] - self.crop_region[0] - self.crop_region[1]

        depth_params = self.cfg["observations"]["policy"][self.depth_obs_names[0]]["params"]
        downsample_factor = depth_params.get("history_skip_frames", depth_params.get("time_downsample_factor", 1))
        downsample_factor = max(int(downsample_factor), 1)
        frames = int(
            depth_params.get(
                "num_output_frames",
                (camera_cfg["data_histories"]["distance_to_image_plane_noised"] - 1) / downsample_factor + 1,
            )
        )
        sim_frequency = int(1 / camera_cfg["update_period"])
        real_downsample_factor = max(int(self.rs_frequency / sim_frequency * downsample_factor), 1)
        self.depth_obs_indices = np.linspace(-1 - real_downsample_factor * (frames - 1), -1, frames).astype(int)
        self.depth_image_buffer = CircularBuffer(length=self.rs_frequency)

    def _load_models(self):
        providers = ["CPUExecutionProvider"]
        depth_encoder_path = self._resolve_logdir_path("exported", "0-depth_encoder.onnx")
        actor_path = self._resolve_logdir_path("exported", "actor.onnx")
        self.ort_sessions["depth_encoder"] = ort.InferenceSession(depth_encoder_path, providers=providers)
        self.ort_sessions["actor"] = ort.InferenceSession(actor_path, providers=providers)
        print(f"Loaded parkour ONNX models from {self.logdir}")

    def reset(self):
        super().reset()

    def step(self):
        proprio_obs = []
        for obs_name in self.proprio_obs_names:
            proprio_obs.append(self._get_single_obs_term(obs_name).reshape(1, -1).astype(np.float32))
        proprio_obs = np.concatenate(proprio_obs, axis=-1)

        depth_obs = (
            self._get_single_obs_term(self.depth_obs_names[0])
            .reshape(1, -1, self.depth_height, self.depth_width)
            .astype(np.float32)
        )
        self._publish_debug_depth(depth_obs)

        depth_input_name = self.ort_sessions["depth_encoder"].get_inputs()[0].name
        depth_embedding = self.ort_sessions["depth_encoder"].run(None, {depth_input_name: depth_obs})[0]
        actor_input = np.concatenate([proprio_obs, depth_embedding], axis=1)
        actor_input_name = self.ort_sessions["actor"].get_inputs()[0].name
        action = self.ort_sessions["actor"].run(None, {actor_input_name: actor_input})[0].reshape(-1)
        action = self._clip_camera_yaw_pitch_action(action.astype(np.float32))
        self._debug_policy_io(action)
        return action, False

    def _clip_camera_yaw_pitch_action(self, action: np.ndarray) -> np.ndarray:
        if not self._camera_action_joint_ids:
            return action
        action = action.copy()
        for joint_id, (low, high) in zip(self._camera_action_joint_ids, self._camera_action_ranges):
            target = action[joint_id] * self.action_scale[joint_id] + self.action_offset[joint_id]
            target = np.clip(target, low, high)
            if abs(self.action_scale[joint_id]) > 1e-8:
                action[joint_id] = (target - self.action_offset[joint_id]) / self.action_scale[joint_id]
        return action

    def _publish_debug_depth(self, depth_obs: np.ndarray):
        if self.debug_depth_publisher is not None:
            depth_image_msg_data = np.asanyarray(depth_obs[0, -1] * 255 * 2, dtype=np.uint16)
            depth_image_msg = rnp.msgify(Image, depth_image_msg_data, encoding="16UC1")
            depth_image_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            depth_image_msg.header.frame_id = "realsense_depth_link"
            self.debug_depth_publisher.publish(depth_image_msg)
        if self.debug_pointcloud_publisher is not None:
            pointcloud_msg = self.ros_node.depth_image_to_pointcloud_msg(
                depth_obs[0, -1] * self.depth_range[1] + self.depth_range[0]
            )
            self.debug_pointcloud_publisher.publish(pointcloud_msg)

    def _debug_policy_io(self, action: np.ndarray):
        if not self.debug_policy_io:
            return
        now = time.time()
        if now - self._last_policy_io_log_time < 0.5:
            return
        self._last_policy_io_log_time = now
        target = action * self.action_scale + self.action_offset
        summary = []
        for joint_name in ("left_knee_joint", "right_knee_joint", "head_yaw_joint", "head_pitch_joint"):
            joint_id = self.ros_node.sim_joint_names.index(joint_name)
            summary.append(f"{joint_name}={target[joint_id]:+.3f}")
        self.ros_node.get_logger().info(
            f"parkour_io speed={self.speed_scale:.3f} raw_max={np.max(np.abs(action)):.3f} "
            f"target[{', '.join(summary)}]"
        )

    def _get_base_velocity_cmd_obs(self):
        raw = float(self.ros_node.joy_stick_data.ly or 0.0)
        if abs(raw) < self.lin_vel_deadband:
            speed_scale = 0.0
        else:
            speed_scale = float(np.clip(raw, 0.0, 1.0))
        self.speed_scale = speed_scale
        self.xyyaw_command = np.array([speed_scale], dtype=np.float32)
        return self.xyyaw_command

    def _get_base_velocity_obs(self):
        return self._get_base_velocity_cmd_obs()

    def _get_joint_vel_rel_obs(self):
        non_head_joint_ids = [
            i
            for i, joint_name in enumerate(self.ros_node.sim_joint_names)
            if joint_name not in ("head_yaw_joint", "head_pitch_joint")
        ]
        return self.ros_node.joint_vel_[non_head_joint_ids] - self.default_joint_vel[non_head_joint_ids]

    def _get_camera_offset_yaw_pitch_obs(self):
        yaw_id = self.ros_node.sim_joint_names.index("head_yaw_joint")
        pitch_id = self.ros_node.sim_joint_names.index("head_pitch_joint")
        return self.ros_node.joint_pos_[[yaw_id, pitch_id]]

    def refresh_depth_frame(self):
        self.ros_node.refresh_rs_data()
        depth_image = cv2.resize(self.ros_node.rs_depth_data, self.output_resolution, interpolation=cv2.INTER_NEAREST)

        up, down, left, right = self.crop_region
        h, w = depth_image.shape
        depth_image = depth_image[up : h - down if down > 0 else h, left : w - right if right > 0 else w]
        depth_image = cv2.inpaint(depth_image, (depth_image < 0.2).astype(np.uint8), 3, cv2.INPAINT_NS)
        if self.gaussian_kernel_size is not None:
            depth_image = cv2.GaussianBlur(depth_image, self.gaussian_kernel_size, self.gaussian_sigma)

        clipped = np.clip(depth_image, self.depth_range[0], self.depth_range[1])
        normalized = (clipped - self.depth_range[0]) / (self.depth_range[1] - self.depth_range[0])
        output = normalized * (self.depth_output_range[1] - self.depth_output_range[0]) + self.depth_output_range[0]
        self.depth_image_buffer.append(output.astype(np.float32))

    def _get_delayed_visualizable_image_obs(self):
        self.refresh_depth_frame()
        return self.depth_image_buffer.buffer[self.depth_obs_indices, ...]


class ParkourStandAgent(ParkourAgent):
    def __init__(self, logdir: str, ros_node: RealNode):
        super().__init__(logdir, ros_node, depth_vis=False, pointcloud_vis=False)

    def _get_delayed_visualizable_image_obs(self):
        return np.zeros((len(self.depth_obs_indices), self.depth_height, self.depth_width), dtype=np.float32)


class Body29DepthOn31Agent(ParkourStandAgent):
    """Run a 29-DoF depth policy on the 31-DoF node while holding head joints fixed."""

    BODY_DOF = 29

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._body_joint_ids = np.arange(self.BODY_DOF, dtype=np.int64)
        self._head_joint_ids = np.arange(self.BODY_DOF, self.ros_node.NUM_ACTIONS, dtype=np.int64)
        self._apply_head_defaults()

    def _parse_action_config(self):
        self._camera_action_joint_ids = []
        self._camera_action_ranges = []
        self.default_joint_pos = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
        for joint_name_expr, joint_pos in self.cfg["scene"]["robot"]["init_state"]["joint_pos"].items():
            for i in range(self.BODY_DOF):
                name = self.ros_node.sim_joint_names[i]
                if re.search(joint_name_expr, name):
                    self.default_joint_pos[i] = joint_pos

        self.default_joint_vel = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
        for joint_name_expr, joint_vel in self.cfg["scene"]["robot"]["init_state"]["joint_vel"].items():
            for i in range(self.BODY_DOF):
                name = self.ros_node.sim_joint_names[i]
                if re.search(joint_name_expr, name):
                    self.default_joint_vel[i] = joint_vel

        self._p_gains = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
        self._d_gains = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
        for actuator_config in self.cfg["scene"]["robot"]["actuators"].values():
            for i in range(self.BODY_DOF):
                name = self.ros_node.sim_joint_names[i]
                for joint_name_expr in actuator_config["joint_names_expr"]:
                    if not re.search(joint_name_expr, name):
                        continue
                    self._p_gains[i] = self._get_config_value_for_joint(actuator_config["stiffness"], name, joint_name_expr)
                    self._d_gains[i] = self._get_config_value_for_joint(actuator_config["damping"], name, joint_name_expr)

        self._action_scale = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
        self._action_offset = self.default_joint_pos.copy()
        for action_config in self.cfg["actions"].values():
            if action_config.get("asset_name") != "robot" or "joint_names" not in action_config:
                continue
            use_default_offset = action_config.get("use_default_offset", True)
            offset = action_config.get("offset", 0.0)
            scale = action_config.get("scale", 1.0)
            for i in range(self.BODY_DOF):
                name = self.ros_node.sim_joint_names[i]
                for joint_name_expr in action_config["joint_names"]:
                    if not re.search(joint_name_expr, name):
                        continue
                    self._action_scale[i] = self._get_config_value_for_joint(scale, name, joint_name_expr)
                    if not use_default_offset:
                        self._action_offset[i] = self._get_config_value_for_joint(offset, name, joint_name_expr)
                    break

    def _apply_head_defaults(self):
        if self.ros_node.NUM_ACTIONS <= self.BODY_DOF:
            return
        head_default = np.array([0.0, 0.8726646259971648], dtype=np.float32)
        self.default_joint_pos[self._head_joint_ids] = head_default[: len(self._head_joint_ids)]
        self._action_offset[self._head_joint_ids] = head_default[: len(self._head_joint_ids)]
        self._action_scale[self._head_joint_ids] = 0.0

    def _get_base_velocity_cmd_obs(self):
        return np.zeros(3, dtype=np.float32)

    def _get_base_velocity_obs(self):
        return self._get_base_velocity_cmd_obs()

    def _get_joint_pos_obs(self):
        return self.ros_node.joint_pos_[self._body_joint_ids]

    def _get_joint_vel_obs(self):
        return self.ros_node.joint_vel_[self._body_joint_ids]

    def _get_joint_pos_rel_obs(self):
        return self.ros_node.joint_pos_[self._body_joint_ids] - self.default_joint_pos[self._body_joint_ids]

    def _get_joint_vel_rel_obs(self):
        return self.ros_node.joint_vel_[self._body_joint_ids] - self.default_joint_vel[self._body_joint_ids]

    def _get_last_action_obs(self):
        return np.asarray(self.ros_node.action, dtype=np.float32)[self._body_joint_ids]

    def _get_camera_offset_yaw_pitch_obs(self):
        return np.zeros(2, dtype=np.float32)

    def _debug_policy_io(self, action: np.ndarray):
        return

    def step(self):
        action, done = super().step()
        full_action = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
        full_action[self._body_joint_ids] = action[: self.BODY_DOF]
        return full_action, done
