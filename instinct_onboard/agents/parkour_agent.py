# from __future__ import annotations

# import math
# import os
# import re
# import time
# from typing import Tuple

# import cv2
# import numpy as np
# import onnxruntime as ort
# import prettytable
# import ros2_numpy as rnp
# import yaml
# from geometry_msgs.msg import TransformStamped
# from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
# from tf2_ros import StaticTransformBroadcaster

# from instinct_onboard import robot_cfgs
# from instinct_onboard.agents.base import OnboardAgent
# from instinct_onboard.ros_nodes.base import RealNode
# from instinct_onboard.utils import CircularBuffer


# class ParkourAgent(OnboardAgent):
#     rs_resolution = (480, 270)
#     rs_frequency = 60

#     def __init__(
#         self,
#         logdir: str,
#         ros_node: RealNode,
#         depth_vis: bool = True,
#         pointcloud_vis: bool = True,
#         initial_speed_scale: float = 0.0,
#         lin_vel_deadband=0.5,
#         lin_vel_range=[0.5, 0.5],
#         ang_vel_deadband=0.15,
#         ang_vel_range=[0.0, 1.0],
#     ):
#         super().__init__(logdir, ros_node)
#         self.ort_sessions = dict()
#         self.speed_scale = float(initial_speed_scale)
#         self.lin_vel_deadband = lin_vel_deadband
#         self.ang_vel_deadband = ang_vel_deadband
#         self.cmd_px_range = lin_vel_range
#         self.cmd_nx_range = [0.0, 0.0]
#         self.cmd_py_range = [0.0, 0.0]
#         self.cmd_ny_range = [0.0, 0.0]
#         self.cmd_pyaw_range = ang_vel_range
#         self.cmd_nyaw_range = ang_vel_range
#         self._parse_obs_config()
#         self._parse_action_config()
#         self._load_models()
#         self.depth_vis = depth_vis
#         if self.depth_vis:
#             self.debug_depth_publisher = self.ros_node.create_publisher(Image, "/debug/depth_image", 10)
#         else:
#             self.debug_depth_publisher = None
#         self.pointcloud_vis = pointcloud_vis
#         if self.pointcloud_vis:
#             self.debug_pointcloud_publisher = self.ros_node.create_publisher(PointCloud2, "/debug/pointcloud", 10)
#         else:
#             self.debug_pointcloud_publisher = None

#     def _parse_obs_config(self):
#         super()._parse_obs_config()
#         with open(self._resolve_logdir_path("params", "agent.yaml")) as f:
#             self.agent_cfg = yaml.unsafe_load(f)
#         all_obs_names = list(self.obs_funcs.keys())
#         self.proprio_obs_names = [obs_name for obs_name in all_obs_names if "depth" not in obs_name]
#         print(f"ParkourAgent proprioception names: {self.proprio_obs_names}")
#         self.depth_obs_names = [obs_name for obs_name in all_obs_names if "depth" in obs_name]
#         assert len(self.depth_obs_names) == 1, "Only support one depth observation for now."
#         print(f"ParkourAgent depth observation names: {self.depth_obs_names}")
#         table = prettytable.PrettyTable()
#         table.field_names = ["Observation Name", "Function"]
#         for obs_name, func in self.obs_funcs.items():
#             table.add_row([obs_name, func.__name__])
#         print("Observation functions:")
#         print(table)
#         self._parse_depth_image_config()

#     def _parse_action_config(self):
#         super()._parse_action_config()
#         self._zero_action_joints = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
#         for action_names, action_config in self.cfg["actions"].items():
#             for i in range(self.ros_node.NUM_JOINTS):
#                 name = self.ros_node.sim_joint_names[i]
#                 if "default_joint_names" in action_config:
#                     for _, joint_name_expr in enumerate(action_config["default_joint_names"]):
#                         if re.search(joint_name_expr, name):
#                             self._zero_action_joints[i] = 1.0

#     def _parse_depth_image_config(self):
#         self.output_resolution = [
#             self.cfg["scene"]["camera"]["pattern_cfg"]["width"],
#             self.cfg["scene"]["camera"]["pattern_cfg"]["height"],
#         ]

#         self.depth_range = self.cfg["scene"]["camera"]["noise_pipeline"]["depth_normalization"]["depth_range"]

#         if self.cfg["scene"]["camera"]["noise_pipeline"]["depth_normalization"]["normalize"]:
#             self.depth_output_range = self.cfg["scene"]["camera"]["noise_pipeline"]["depth_normalization"][
#                 "output_range"
#             ]
#         else:
#             self.depth_output_range = self.depth_range

#         if "crop_and_resize" in self.cfg["scene"]["camera"]["noise_pipeline"]:
#             self.crop_region = self.cfg["scene"]["camera"]["noise_pipeline"]["crop_and_resize"]["crop_region"]
#         if "gaussian_blur" in self.cfg["scene"]["camera"]["noise_pipeline"]:
#             self.gaussian_kernel_size = (
#                 self.cfg["scene"]["camera"]["noise_pipeline"]["gaussian_blur"]["kernel_size"],
#                 self.cfg["scene"]["camera"]["noise_pipeline"]["gaussian_blur"]["kernel_size"],
#             )
#             self.gaussian_sigma = self.cfg["scene"]["camera"]["noise_pipeline"]["gaussian_blur"]["sigma"]
#         if "blind_spot" in self.cfg["scene"]["camera"]["noise_pipeline"]:
#             self.blind_spot_crop = self.cfg["scene"]["camera"]["noise_pipeline"]["blind_spot"]["crop_region"]
#         self.depth_width = (
#             self.output_resolution[0] - self.crop_region[2] - self.crop_region[3]
#             if hasattr(self, "crop_region")
#             else self.output_resolution[0]
#         )
#         self.depth_height = (
#             self.output_resolution[1] - self.crop_region[0] - self.crop_region[1]
#             if hasattr(self, "crop_region")
#             else self.output_resolution[1]
#         )
#         # For sample resize
#         square_size = int(self.rs_resolution[0] // self.output_resolution[0])
#         rows, cols = self.rs_resolution[1], self.rs_resolution[0]
#         center_y_coords = np.arange(self.output_resolution[1]) * square_size + square_size // 2
#         center_x_coords = np.arange(self.output_resolution[0]) * square_size + square_size // 2
#         y_grid, x_grid = np.meshgrid(center_y_coords, center_x_coords, indexing="ij")
#         valid_mask = (y_grid < rows) & (x_grid < cols)
#         self.y_valid = np.clip(y_grid, 0, rows - 1)
#         self.x_valid = np.clip(x_grid, 0, cols - 1)
#         # For downsample history
#                # For downsample history
#         if "history_skip_frames" in self.cfg["observations"]["policy"]["depth_image"]["params"]:
#             downsample_factor = self.cfg["observations"]["policy"]["depth_image"]["params"]["history_skip_frames"]
#         else:
#             downsample_factor = self.cfg["observations"]["policy"]["depth_image"]["params"]["time_downsample_factor"]
#         downsample_factor = max(int(downsample_factor), 1)
#         depth_obs_params = self.cfg["observations"]["policy"]["depth_image"]["params"]
#         if "num_output_frames" in depth_obs_params:
#             frames = int(depth_obs_params["num_output_frames"])
#         else:
#             frames = int(
#                 (self.cfg["scene"]["camera"]["data_histories"]["distance_to_image_plane_noised"] - 1)
#                 / downsample_factor
#                 + 1
#             )
#         sim_frequency = int(1 / self.cfg["scene"]["camera"]["update_period"])
#         real_downsample_factor = int(self.rs_frequency / sim_frequency * downsample_factor)
#         self.depth_obs_indices = np.linspace(-1 - real_downsample_factor * (frames - 1), -1, frames).astype(int)
#         sim_frequency = int(1 / self.cfg["scene"]["camera"]["update_period"])
#         real_downsample_factor = int(self.rs_frequency / sim_frequency * downsample_factor)
#         self.depth_obs_indices = np.linspace(-1 - real_downsample_factor * (frames - 1), -1, frames).astype(int)
#         print(f"Depth observation downsample indices: {self.depth_obs_indices}")
#         self.depth_image_buffer = CircularBuffer(length=self.rs_frequency)

#     def _parse_observation_function(self, obs_name, obs_config):
#         obs_func = obs_config["func"].split(":")[-1]  # get the function name from the config
#         if obs_func == "depth_image":
#             obs_name = "depth_latent"
#             if hasattr(self, f"_get_{obs_name}_obs"):
#                 self.obs_funcs[obs_name] = getattr(self, f"_get_{obs_name}_obs")
#                 return
#             else:
#                 raise ValueError(f"Unknown observation function for observation {obs_name}")
#         self.xyyaw_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
#         return super()._parse_observation_function(obs_name, obs_config)

#     def _load_models(self):
#         """Load the ONNX model for the agent."""
#         # load ONNX models
#         ort_execution_providers = ort.get_available_providers()
#         depth_encoder_path = self._resolve_logdir_path("exported", "0-depth_encoder.onnx")
#         self.ort_sessions["depth_encoder"] = ort.InferenceSession(depth_encoder_path, providers=ort_execution_providers)
#         actor_path = self._resolve_logdir_path("exported", "actor.onnx")
#         self.ort_sessions["actor"] = ort.InferenceSession(actor_path, providers=ort_execution_providers)
#         print(f"Loaded ONNX models from {self.logdir}")

#     def reset(self):
#         """Reset the agent state and the rosbag reader."""
#         pass

#     def step(self):
#         """Perform a single step of the agent."""
#         # pack actor MLP input
#         proprio_obs = []
#         for proprio_obs_name in self.proprio_obs_names:
#             obs_term_value = self._get_single_obs_term(proprio_obs_name)
#             proprio_obs.append(np.reshape(obs_term_value, (1, -1)).astype(np.float32))
#         proprio_obs = np.concatenate(proprio_obs, axis=-1)

#         depth_obs = (
#             self._get_single_obs_term(self.depth_obs_names[0])
#             .reshape(1, -1, self.depth_height, self.depth_width)
#             .astype(np.float32)
#         )
#         # if self.depth_vis:
#         #     self._vis_depth_obs(depth_obs.reshape(-1, self.depth_height, self.depth_width))
#         if self.debug_depth_publisher is not None:
#             # NOTE: the +5.0 is a empirical value to ensure the normalized obs is not negative.
#             # Not using normalizer's value is to prevent further visualization code bugs.
#             depth_image_msg_data = np.asanyarray(
#                 depth_obs[0, -1].reshape(self.depth_height, self.depth_width) * 255 * 2,
#                 dtype=np.uint16,
#             )
#             depth_image_msg = rnp.msgify(Image, depth_image_msg_data, encoding="16UC1")
#             depth_image_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
#             depth_image_msg.header.frame_id = "realsense_depth_link"
#             self.debug_depth_publisher.publish(depth_image_msg)
#         if self.debug_pointcloud_publisher is not None:
#             pointcloud_msg = self.ros_node.depth_image_to_pointcloud_msg(
#                 depth_obs[0, -1].reshape(self.depth_height, self.depth_width) * self.depth_range[1]
#                 + self.depth_range[0]
#             )
#             self.debug_pointcloud_publisher.publish(pointcloud_msg)

#         depth_image_output = self.ort_sessions["depth_encoder"].run(
#             None, {self.ort_sessions["depth_encoder"].get_inputs()[0].name: depth_obs}
#         )[0]
#         # run actor MLP
#         actor_input = np.concatenate([proprio_obs, depth_image_output], axis=1)
#         actor_input_name = self.ort_sessions["actor"].get_inputs()[0].name
#         action = self.ort_sessions["actor"].run(None, {actor_input_name: actor_input})[0]
#         action = action.reshape(-1)
#         #action = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
#         # reconstruct full action including zeroed joints
#         mask = (self._zero_action_joints == 0).astype(bool)
#         full_action = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
#         full_action[mask] = action

#         return full_action, False

#     """
#     Agent specific observation functions for Parkour Agent.
#     """

#     def set_speed_scale(self, speed_scale: float):
#         self.speed_scale = float(np.clip(speed_scale, 0.0, 1.0))

#     def _get_base_velocity_obs(self):
#         """Return the normalized forward speed scale used during training."""
#         #joystick_scale = max(self.speed_scale, float(np.clip(self.ros_node.joy_stick_data.ly, 0.0, 1.0)))
#         joystick_scale = float(np.clip(self.ros_node.joy_stick_data.ly, 0.0, 1.0))
#         self.speed_scale = joystick_scale
#         self.xyyaw_command = np.array([joystick_scale], dtype=np.float32)
#         return self.xyyaw_command

#     def _get_joint_vel_rel_obs(self):
#         """Return shape: (num_non_head_joints,) = (29,)
#         Simulation's joint_vel uses NON_HEAD_JOINT_REGEX which excludes head_yaw_joint
#         and head_pitch_joint. The policy was trained with 29-dim velocity observations.
#         We return only the non-head joint velocities to match the training setup.
#         """
#         non_head_joint_ids = [
#             i
#             for i, joint_name in enumerate(self.ros_node.sim_joint_names)
#             if joint_name not in ("head_yaw_joint", "head_pitch_joint")
#         ]
#         return self.ros_node.joint_vel_[non_head_joint_ids]  # shape (29,)

#     def _get_camera_offset_yaw_pitch_obs(self):
#         """Return camera yaw/pitch in simulation coordinates, matching mdp.camera_offset_yaw_pitch."""
#         yaw_id = self.ros_node.sim_joint_names.index("head_yaw_joint")
#         pitch_id = self.ros_node.sim_joint_names.index("head_pitch_joint")
#         return self.ros_node.joint_pos_[[yaw_id, pitch_id]]

#     def _get_last_action_obs(self):
#         """Return shape: (num_active_joints,)"""
#         actions = np.asarray(self.ros_node.action).astype(np.float32)
#         mask = (1.0 - self._zero_action_joints).astype(bool)
#         return actions[mask]

#     def refresh_depth_frame(self):
#         """Return the depth image."""
#         self.ros_node.refresh_rs_data()
#         depth_image_np: np.ndarray = self.ros_node.rs_depth_data
#         # normalize based on given range
#         depth_image = cv2.resize(depth_image_np, self.output_resolution, interpolation=cv2.INTER_NEAREST)

#         if hasattr(self, "crop_region"):
#             shape = depth_image.shape
#             x1, x2, y1, y2 = self.crop_region
#             depth_image = depth_image[x1 : shape[0] - x2, y1 : shape[1] - y2]

#         mask = (depth_image < 0.2).astype(np.uint8)
#         depth_image = cv2.inpaint(depth_image, mask, 3, cv2.INPAINT_NS)

#         if hasattr(self, "blind_spot_crop"):
#             shape = depth_image.shape
#             x1, x2, y1, y2 = self.blind_spot_crop
#             depth_image[:x1, :] = 0
#             depth_image[shape[0] - x2 :, :] = 0
#             depth_image[:, :y1] = 0
#             depth_image[:, shape[1] - y2 :] = 0
#         if hasattr(self, "gaussian_kernel_size"):
#             depth_image = cv2.GaussianBlur(
#                 depth_image, self.gaussian_kernel_size, self.gaussian_sigma, self.gaussian_sigma
#             )

#         filt_m = np.clip(depth_image, self.depth_range[0], self.depth_range[1])
#         filt_norm = (filt_m - self.depth_range[0]) / (self.depth_range[1] - self.depth_range[0])

#         output_norm = filt_norm * (self.depth_output_range[1] - self.depth_output_range[0]) + self.depth_output_range[0]
#         self.depth_image_buffer.append(output_norm)

#     def _get_delayed_visualizable_image_obs(self):
#         return self._get_depth_image_downsample_obs()

#     def _get_depth_image_downsample_obs(self):
#         self.refresh_depth_frame()
#         return self.depth_image_buffer.buffer[self.depth_obs_indices, ...]

#     def _vis_depth_obs(self, depth_obs: np.ndarray):
#         depth_tiles = (np.clip(depth_obs, 0.0, 1.0) * 255).astype(np.uint8)
#         rows, cols = 2, 4
#         tile_h, tile_w = depth_tiles.shape[1], depth_tiles.shape[2]
#         grid = np.zeros((rows * tile_h, cols * tile_w), dtype=np.uint8)
#         for idx in range(depth_tiles.shape[0]):
#             r, c = divmod(idx, cols)
#             grid[r * tile_h : (r + 1) * tile_h, c * tile_w : (c + 1) * tile_w] = depth_tiles[idx]
#         cv2.imwrite("depth_obs_grid.png", grid)


# class ParkourStandAgent(ParkourAgent):
#     def __init__(
#         self,
#         logdir: str,
#         ros_node: RealNode,
#     ):
#         super().__init__(logdir, ros_node, depth_vis=False, pointcloud_vis=False)

#     def _get_depth_image_downsample_obs(self):
#         return np.zeros([len(self.depth_obs_indices), self.depth_height, self.depth_width])


# class Body29DepthOn31Agent(ParkourStandAgent):
#     """Run the original 29-DoF stand path on a 31-DoF onboard node."""

#     BODY_DOF = 29

#     def __init__(
#         self,
#         logdir: str,
#         ros_node: RealNode,
#         x_vel_scale: float = 0.5,
#         y_vel_scale: float = 0.5,
#         yaw_vel_scale: float = 1.0,
#     ):
#         self.x_vel_scale = x_vel_scale
#         self.y_vel_scale = y_vel_scale
#         self.yaw_vel_scale = yaw_vel_scale
#         super().__init__(logdir=logdir, ros_node=ros_node)
#         self._body_joint_ids = np.arange(self.BODY_DOF, dtype=np.int64)
#         self._head_joint_ids = np.arange(self.BODY_DOF, self.ros_node.NUM_ACTIONS, dtype=np.int64)
#         self._apply_head_defaults()

#     def _parse_action_config(self):
#         self.default_joint_pos = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
#         for joint_name_expr, joint_pos in self.cfg["scene"]["robot"]["init_state"]["joint_pos"].items():
#             for i in range(self.BODY_DOF):
#                 name = self.ros_node.sim_joint_names[i]
#                 if re.search(joint_name_expr, name):
#                     self.default_joint_pos[i] = joint_pos

#         self.default_joint_vel = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
#         for joint_name_expr, joint_vel in self.cfg["scene"]["robot"]["init_state"]["joint_vel"].items():
#             for i in range(self.BODY_DOF):
#                 name = self.ros_node.sim_joint_names[i]
#                 if re.search(joint_name_expr, name):
#                     self.default_joint_vel[i] = joint_vel

#         self._p_gains = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
#         self._d_gains = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
#         for actuator_config in self.cfg["scene"]["robot"]["actuators"].values():
#             for i in range(self.BODY_DOF):
#                 name = self.ros_node.sim_joint_names[i]
#                 for joint_name_expr in actuator_config["joint_names_expr"]:
#                     if not re.search(joint_name_expr, name):
#                         continue
#                     if isinstance(actuator_config["stiffness"], dict):
#                         for key, value in actuator_config["stiffness"].items():
#                             if re.search(key, name):
#                                 self._p_gains[i] = value
#                     else:
#                         self._p_gains[i] = actuator_config["stiffness"]
#                     if isinstance(actuator_config["damping"], dict):
#                         for key, value in actuator_config["damping"].items():
#                             if re.search(key, name):
#                                 self._d_gains[i] = value
#                     else:
#                         self._d_gains[i] = actuator_config["damping"]

#         self._action_scale = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
#         self._action_offset = self.default_joint_pos.copy()
#         action_config = self.cfg["actions"]["joint_pos"]
#         use_default_offset = action_config.get("use_default_offset", True)
#         offset = action_config.get("offset", 0.0)
#         scale = action_config.get("scale", 1.0)
#         for i in range(self.BODY_DOF):
#             name = self.ros_node.sim_joint_names[i]
#             for joint_name_expr in action_config["joint_names"]:
#                 if not re.search(joint_name_expr, name):
#                     continue
#                 self._action_scale[i] = self._get_config_value_for_joint(scale, name, joint_name_expr)
#                 if not use_default_offset:
#                     self._action_offset[i] = self._get_config_value_for_joint(offset, name, joint_name_expr)
#                 break
#         self._zero_action_joints = np.zeros(self.BODY_DOF, dtype=np.float32)
#     def _apply_head_defaults(self):
#         if self.ros_node.NUM_ACTIONS <= self.BODY_DOF:
#             return
#         head_default = robot_cfgs.G1_31Dof_TorsoBase.head_default_joint_pos
#         self.default_joint_pos[self._head_joint_ids] = head_default[: len(self._head_joint_ids)]
#         self._action_offset[self._head_joint_ids] = head_default[: len(self._head_joint_ids)]
#         self._action_scale[self._head_joint_ids] = 0.0
#     def _get_base_velocity_obs(self):
#         x_vel = self.ros_node.joy_stick_data.ly * self.x_vel_scale
#         y_vel = -self.ros_node.joy_stick_data.lx * self.y_vel_scale
#         yaw_vel = -self.ros_node.joy_stick_data.rx * self.yaw_vel_scale
#         self.xyyaw_command = np.array([x_vel, y_vel, yaw_vel], dtype=np.float32)
#         return self.xyyaw_command

#     def _get_joint_pos_obs(self):
#         return self.ros_node.joint_pos_[self._body_joint_ids]

#     def _get_joint_vel_obs(self):
#         return self.ros_node.joint_vel_[self._body_joint_ids]

#     def _get_joint_pos_rel_obs(self):
#         return self.ros_node.joint_pos_[self._body_joint_ids] - self.default_joint_pos[self._body_joint_ids]

#     def _get_joint_vel_rel_obs(self):
#         return self.ros_node.joint_vel_[self._body_joint_ids] - self.default_joint_vel[self._body_joint_ids]

#     def _get_last_action_obs(self):
#         return np.asarray(self.ros_node.action, dtype=np.float32)[self._body_joint_ids]

#     def step(self):
#         proprio_obs = []
#         for proprio_obs_name in self.proprio_obs_names:
#             obs_term_value = self._get_single_obs_term(proprio_obs_name)
#             proprio_obs.append(np.reshape(obs_term_value, (1, -1)).astype(np.float32))
#         proprio_obs = np.concatenate(proprio_obs, axis=-1)

#         depth_obs = (
#             self._get_single_obs_term(self.depth_obs_names[0])
#             .reshape(1, -1, self.depth_height, self.depth_width)
#             .astype(np.float32)
#         )
#         depth_image_output = self.ort_sessions["depth_encoder"].run(
#             None, {self.ort_sessions["depth_encoder"].get_inputs()[0].name: depth_obs}
#         )[0]
#         actor_input = np.concatenate([proprio_obs, depth_image_output], axis=1)
#         actor_input_name = self.ort_sessions["actor"].get_inputs()[0].name
#         body_action = self.ort_sessions["actor"].run(None, {actor_input_name: actor_input})[0].reshape(-1)

#         full_action = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
#         full_action[self._body_joint_ids] = body_action[: self.BODY_DOF]
#         return full_action, False
from __future__ import annotations

import math
import os
import re
import time
from typing import Tuple

import cv2
import numpy as np
import onnxruntime as ort
import prettytable
import ros2_numpy as rnp
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from instinct_onboard import robot_cfgs
from instinct_onboard.agents.base import OnboardAgent
from instinct_onboard.ros_nodes.base import RealNode
from instinct_onboard.utils import CircularBuffer


class ParkourAgent(OnboardAgent):
    rs_resolution = (480, 270)
    rs_frequency = 60

    def __init__(
        self,
        logdir: str,
        ros_node: RealNode,
        depth_vis: bool = True,
        pointcloud_vis: bool = True,
        initial_speed_scale: float = 0.0,
        debug_policy_io: bool = False,
        lock_head_to_default: bool = False,
        lin_vel_deadband=0.5,
        lin_vel_range=[0.5, 0.5],
        ang_vel_deadband=0.15,
        ang_vel_range=[0.0, 1.0],
    ):
        super().__init__(logdir, ros_node)
        self.ort_sessions = dict()
        self.speed_scale = float(initial_speed_scale)
        self.current_speed_scale = self.speed_scale
        self.debug_policy_io = debug_policy_io
        self.lock_head_to_default = lock_head_to_default
        self._last_policy_io_log_time = 0.0
        self.lin_vel_deadband = lin_vel_deadband
        self.ang_vel_deadband = ang_vel_deadband
        self.cmd_px_range = lin_vel_range
        self.cmd_nx_range = [0.0, 0.0]
        self.cmd_py_range = [0.0, 0.0]
        self.cmd_ny_range = [0.0, 0.0]
        self.cmd_pyaw_range = ang_vel_range
        self.cmd_nyaw_range = ang_vel_range
        self._parse_obs_config()
        self._parse_action_config()
        self._load_models()
        self.depth_vis = depth_vis
        if self.depth_vis:
            self.debug_depth_publisher = self.ros_node.create_publisher(Image, "/debug/depth_image", 10)
        else:
            self.debug_depth_publisher = None
        self.pointcloud_vis = pointcloud_vis
        if self.pointcloud_vis:
            self.debug_pointcloud_publisher = self.ros_node.create_publisher(PointCloud2, "/debug/pointcloud", 10)
        else:
            self.debug_pointcloud_publisher = None

    def _parse_obs_config(self):
        super()._parse_obs_config()
        with open(self._resolve_logdir_path("params", "agent.yaml")) as f:
            self.agent_cfg = yaml.unsafe_load(f)
        all_obs_names = list(self.obs_funcs.keys())
        self.proprio_obs_names = [obs_name for obs_name in all_obs_names if "depth" not in obs_name]
        print(f"ParkourAgent proprioception names: {self.proprio_obs_names}")
        self.depth_obs_names = [obs_name for obs_name in all_obs_names if "depth" in obs_name]
        assert len(self.depth_obs_names) == 1, "Only support one depth observation for now."
        print(f"ParkourAgent depth observation names: {self.depth_obs_names}")
        table = prettytable.PrettyTable()
        table.field_names = ["Observation Name", "Function"]
        for obs_name, func in self.obs_funcs.items():
            table.add_row([obs_name, func.__name__])
        print("Observation functions:")
        print(table)
        self._parse_depth_image_config()

    def _parse_action_config(self):
        super()._parse_action_config()
        self._camera_action_joint_ids = None
        self._camera_action_low = None
        self._camera_action_high = None
        self._zero_action_joints = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
        for action_names, action_config in self.cfg["actions"].items():
            if "yaw_joint_name" in action_config and "pitch_joint_name" in action_config:
                yaw_id = self.ros_node.sim_joint_names.index(action_config["yaw_joint_name"])
                pitch_id = self.ros_node.sim_joint_names.index(action_config["pitch_joint_name"])
                self._camera_action_joint_ids = np.array([yaw_id, pitch_id], dtype=np.int64)
                self._camera_action_low = np.array(
                    [action_config["yaw_range"][0], action_config["pitch_range"][0]],
                    dtype=np.float32,
                )
                self._camera_action_high = np.array(
                    [action_config["yaw_range"][1], action_config["pitch_range"][1]],
                    dtype=np.float32,
                )
                gimbal_low, gimbal_high = self._get_gimbal_action_limits()
                if gimbal_low is not None and gimbal_high is not None:
                    self._camera_action_low = np.maximum(self._camera_action_low, gimbal_low)
                    self._camera_action_high = np.minimum(self._camera_action_high, gimbal_high)
            for i in range(self.ros_node.NUM_JOINTS):
                name = self.ros_node.sim_joint_names[i]
                if "default_joint_names" in action_config:
                    for _, joint_name_expr in enumerate(action_config["default_joint_names"]):
                        if re.search(joint_name_expr, name):
                            self._zero_action_joints[i] = 1.0

    def _get_gimbal_action_limits(self):
        if not hasattr(self.ros_node, "gimbal_pan_range") or not hasattr(self.ros_node, "gimbal_tilt_range"):
            return None, None

        def servo_deg_range_to_sim_rad(range_deg, sign):
            values = np.deg2rad(np.asarray(range_deg, dtype=np.float32)) * float(sign)
            return float(np.min(values)), float(np.max(values))

        yaw_low, yaw_high = servo_deg_range_to_sim_rad(self.ros_node.gimbal_pan_range, self.ros_node.joint_signs[29])
        pitch_low, pitch_high = servo_deg_range_to_sim_rad(
            self.ros_node.gimbal_tilt_range,
            self.ros_node.joint_signs[30],
        )
        return (
            np.array([yaw_low, pitch_low], dtype=np.float32),
            np.array([yaw_high, pitch_high], dtype=np.float32),
        )

    def _clip_camera_yaw_pitch_action(self, action: np.ndarray) -> np.ndarray:
        """Match CameraYawPitchActionCfg target clipping used during simulation."""
        if self._camera_action_joint_ids is None:
            return action
        joint_ids = self._camera_action_joint_ids
        target = action[joint_ids] * self._action_scale[joint_ids] + self._action_offset[joint_ids]
        target = np.clip(target, self._camera_action_low, self._camera_action_high)
        scale = self._action_scale[joint_ids]
        nonzero = np.abs(scale) > 1e-8
        action[joint_ids[nonzero]] = (
            (target[nonzero] - self._action_offset[joint_ids[nonzero]]) / scale[nonzero]
        )
        return action

    def _lock_head_action_to_default(self, action: np.ndarray) -> np.ndarray:
        """Force camera/head action to the same fixed target used by body-only modes."""
        if not self.lock_head_to_default or self._camera_action_joint_ids is None:
            return action
        joint_ids = self._camera_action_joint_ids
        target = robot_cfgs.G1_31Dof_TorsoBase.head_default_joint_pos[: len(joint_ids)].astype(np.float32)
        scale = self._action_scale[joint_ids]
        nonzero = np.abs(scale) > 1e-8
        action[joint_ids[nonzero]] = (
            (target[nonzero] - self._action_offset[joint_ids[nonzero]]) / scale[nonzero]
        )
        action[joint_ids[~nonzero]] = 0.0
        return action

    def _parse_depth_image_config(self):
        self.output_resolution = (
            int(self.cfg["scene"]["camera"]["pattern_cfg"]["width"]),
            int(self.cfg["scene"]["camera"]["pattern_cfg"]["height"]),
        )

        self.depth_range = self.cfg["scene"]["camera"]["noise_pipeline"]["depth_normalization"]["depth_range"]

        if self.cfg["scene"]["camera"]["noise_pipeline"]["depth_normalization"]["normalize"]:
            self.depth_output_range = self.cfg["scene"]["camera"]["noise_pipeline"]["depth_normalization"][
                "output_range"
            ]
        else:
            self.depth_output_range = self.depth_range

        if "crop_and_resize" in self.cfg["scene"]["camera"]["noise_pipeline"]:
            self.crop_region = self.cfg["scene"]["camera"]["noise_pipeline"]["crop_and_resize"]["crop_region"]
        if "gaussian_blur" in self.cfg["scene"]["camera"]["noise_pipeline"]:
            self.gaussian_kernel_size = (
                self.cfg["scene"]["camera"]["noise_pipeline"]["gaussian_blur"]["kernel_size"],
                self.cfg["scene"]["camera"]["noise_pipeline"]["gaussian_blur"]["kernel_size"],
            )
            self.gaussian_sigma = self.cfg["scene"]["camera"]["noise_pipeline"]["gaussian_blur"]["sigma"]
        if "blind_spot" in self.cfg["scene"]["camera"]["noise_pipeline"]:
            self.blind_spot_crop = self.cfg["scene"]["camera"]["noise_pipeline"]["blind_spot"]["crop_region"]
        self.depth_width = (
            self.output_resolution[0] - self.crop_region[2] - self.crop_region[3]
            if hasattr(self, "crop_region")
            else self.output_resolution[0]
        )
        self.depth_height = (
            self.output_resolution[1] - self.crop_region[0] - self.crop_region[1]
            if hasattr(self, "crop_region")
            else self.output_resolution[1]
        )
        # For sample resize
        square_size = int(self.rs_resolution[0] // self.output_resolution[0])
        rows, cols = self.rs_resolution[1], self.rs_resolution[0]
        center_y_coords = np.arange(self.output_resolution[1]) * square_size + square_size // 2
        center_x_coords = np.arange(self.output_resolution[0]) * square_size + square_size // 2
        y_grid, x_grid = np.meshgrid(center_y_coords, center_x_coords, indexing="ij")
        valid_mask = (y_grid < rows) & (x_grid < cols)
        self.y_valid = np.clip(y_grid, 0, rows - 1)
        self.x_valid = np.clip(x_grid, 0, cols - 1)
        # For downsample history
        if "history_skip_frames" in self.cfg["observations"]["policy"]["depth_image"]["params"]:
            downsample_factor = self.cfg["observations"]["policy"]["depth_image"]["params"]["history_skip_frames"]
        else:
            downsample_factor = self.cfg["observations"]["policy"]["depth_image"]["params"]["time_downsample_factor"]
        downsample_factor = max(int(downsample_factor), 1)
        depth_obs_params = self.cfg["observations"]["policy"]["depth_image"]["params"]
        if "num_output_frames" in depth_obs_params:
            frames = int(depth_obs_params["num_output_frames"])
        else:
            frames = int(
                (self.cfg["scene"]["camera"]["data_histories"]["distance_to_image_plane_noised"] - 1)
                / downsample_factor
                + 1
            )
        sim_frequency = int(1 / self.cfg["scene"]["camera"]["update_period"])
        real_downsample_factor = int(self.rs_frequency / sim_frequency * downsample_factor)
        self.depth_obs_indices = np.linspace(-1 - real_downsample_factor * (frames - 1), -1, frames).astype(int)
        print(f"Depth observation downsample indices: {self.depth_obs_indices}")
        self.depth_image_buffer = CircularBuffer(length=self.rs_frequency)

    def _parse_observation_function(self, obs_name, obs_config):
        obs_func = obs_config["func"].split(":")[-1]  # get the function name from the config
        if obs_func == "depth_image":
            obs_name = "depth_latent"
            if hasattr(self, f"_get_{obs_name}_obs"):
                self.obs_funcs[obs_name] = getattr(self, f"_get_{obs_name}_obs")
                return
            else:
                raise ValueError(f"Unknown observation function for observation {obs_name}")
        self.xyyaw_command = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        return super()._parse_observation_function(obs_name, obs_config)

    def _load_models(self):
        """Load the ONNX model for the agent."""
        # load ONNX models
        ort_execution_providers = ort.get_available_providers()
        depth_encoder_path = self._resolve_logdir_path("exported", "0-depth_encoder.onnx")
        self.ort_sessions["depth_encoder"] = ort.InferenceSession(depth_encoder_path, providers=ort_execution_providers)
        actor_path = self._resolve_logdir_path("exported", "actor.onnx")
        self.ort_sessions["actor"] = ort.InferenceSession(actor_path, providers=ort_execution_providers)
        print(f"Loaded ONNX models from {self.logdir}")

    def reset(self):
        """Reset the agent state and the rosbag reader."""
        pass

    def step(self):
        """Perform a single step of the agent."""
        # pack actor MLP input
        proprio_obs = []
        for proprio_obs_name in self.proprio_obs_names:
            obs_term_value = self._get_single_obs_term(proprio_obs_name)
            proprio_obs.append(np.reshape(obs_term_value, (1, -1)).astype(np.float32))
        proprio_obs = np.concatenate(proprio_obs, axis=-1)

        depth_obs = (
            self._get_single_obs_term(self.depth_obs_names[0])
            .reshape(1, -1, self.depth_height, self.depth_width)
            .astype(np.float32)
        )
        if hasattr(self.ros_node, "show_debug_depth_image"):
            self.ros_node.show_debug_depth_image(depth_obs[0, -1])
        # if self.depth_vis:
        #     self._vis_depth_obs(depth_obs.reshape(-1, self.depth_height, self.depth_width))
        if self.debug_depth_publisher is not None:
            # NOTE: the +5.0 is a empirical value to ensure the normalized obs is not negative.
            # Not using normalizer's value is to prevent further visualization code bugs.
            depth_image_msg_data = np.asanyarray(
                depth_obs[0, -1].reshape(self.depth_height, self.depth_width) * 255 * 2,
                dtype=np.uint16,
            )
            depth_image_msg = rnp.msgify(Image, depth_image_msg_data, encoding="16UC1")
            depth_image_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            depth_image_msg.header.frame_id = "realsense_depth_link"
            self.debug_depth_publisher.publish(depth_image_msg)
        if self.debug_pointcloud_publisher is not None:
            pointcloud_msg = self.ros_node.depth_image_to_pointcloud_msg(
                depth_obs[0, -1].reshape(self.depth_height, self.depth_width) * self.depth_range[1]
                + self.depth_range[0]
            )
            self.debug_pointcloud_publisher.publish(pointcloud_msg)

        depth_image_output = self.ort_sessions["depth_encoder"].run(
            None, {self.ort_sessions["depth_encoder"].get_inputs()[0].name: depth_obs}
        )[0]
        # run actor MLP
        actor_input = np.concatenate([proprio_obs, depth_image_output], axis=1)
        actor_input_name = self.ort_sessions["actor"].get_inputs()[0].name
        action = self.ort_sessions["actor"].run(None, {actor_input_name: actor_input})[0]
        action = action.reshape(-1)
        # reconstruct full action including zeroed joints
        mask = (self._zero_action_joints == 0).astype(bool)
        full_action = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
        full_action[mask] = action
        full_action = self._clip_camera_yaw_pitch_action(full_action)
        full_action = self._lock_head_action_to_default(full_action)
        self._record_policy_io(
            "depth_encoder_actor",
            {
                "proprio_obs": proprio_obs,
                "depth_obs": depth_obs,
                "depth_embedding": depth_image_output,
                "actor_input": actor_input,
                "raw_action": action,
                "full_action": full_action,
                "target_joint_pos": full_action * self.action_scale + self.action_offset,
            },
        )
        self._debug_policy_io(full_action)

        return full_action, False

    def _debug_policy_io(self, action: np.ndarray):
        if not self.debug_policy_io:
            return
        now = time.time()
        if now - self._last_policy_io_log_time < 0.5:
            return
        self._last_policy_io_log_time = now

        target_joint_pos = action * self.action_scale + self.action_offset
        joint_indices = {
            "l_knee": "left_knee_joint",
            "r_knee": "right_knee_joint",
            "l_ankle_p": "left_ankle_pitch_joint",
            "r_ankle_p": "right_ankle_pitch_joint",
            "head_yaw": "head_yaw_joint",
            "head_pitch": "head_pitch_joint",
        }
        target_summary = []
        action_summary = []
        for label, joint_name in joint_indices.items():
            if joint_name not in self.ros_node.sim_joint_names:
                continue
            joint_id = self.ros_node.sim_joint_names.index(joint_name)
            target_summary.append(f"{label}={target_joint_pos[joint_id]:+.3f}")
            action_summary.append(f"{label}={action[joint_id]:+.3f}")
        self.ros_node.get_logger().info(
            "parkour_io "
            f"ly={float(self.ros_node.joy_stick_data.ly):+.3f} "
            f"scale={float(self.current_speed_scale):+.3f} "
            f"raw_max={float(np.max(np.abs(action))):.3f} "
            f"target_max={float(np.max(np.abs(target_joint_pos))):.3f} "
            f"raw[{', '.join(action_summary)}] "
            f"target[{', '.join(target_summary)}]",
        )

    """
    Agent specific observation functions for Parkour Agent.
    """

    def set_speed_scale(self, speed_scale: float):
        self.speed_scale = float(np.clip(speed_scale, 0.0, 1.0))

    def _get_base_velocity_obs(self):
        """Return the normalized forward speed scale used during training."""
        joystick_scale = float(np.clip(self.ros_node.joy_stick_data.ly, 0.0, 1.0))
        self.current_speed_scale = max(self.speed_scale, joystick_scale)
        self.xyyaw_command = np.array([self.current_speed_scale], dtype=np.float32)
        return self.xyyaw_command

    def _get_joint_vel_rel_obs(self):
        """Return shape: (num_non_head_joints,) = (29,)
        Simulation's joint_vel uses NON_HEAD_JOINT_REGEX which excludes head_yaw_joint
        and head_pitch_joint. The policy was trained with 29-dim velocity observations.
        We return only the non-head joint velocities to match the training setup.
        """
        non_head_joint_ids = [
            i
            for i, joint_name in enumerate(self.ros_node.sim_joint_names)
            if joint_name not in ("head_yaw_joint", "head_pitch_joint")
        ]
        return self.ros_node.joint_vel_[non_head_joint_ids]  # shape (29,)

    def _get_camera_offset_yaw_pitch_obs(self):
        """Return camera yaw/pitch in simulation coordinates, matching mdp.camera_offset_yaw_pitch."""
        yaw_id = self.ros_node.sim_joint_names.index("head_yaw_joint")
        pitch_id = self.ros_node.sim_joint_names.index("head_pitch_joint")
        return self.ros_node.joint_pos_[[yaw_id, pitch_id]]

    def _get_last_action_obs(self):
        """Return shape: (num_active_joints,)"""
        actions = np.asarray(self.ros_node.action).astype(np.float32)
        mask = (1.0 - self._zero_action_joints).astype(bool)
        return actions[mask]

    def refresh_depth_frame(self):
        """Return the depth image."""
        self.ros_node.refresh_rs_data()
        depth_image_np: np.ndarray = self.ros_node.rs_depth_data
        # normalize based on given range
        depth_image = cv2.resize(depth_image_np, self.output_resolution, interpolation=cv2.INTER_NEAREST)

        if hasattr(self, "crop_region"):
            shape = depth_image.shape
            x1, x2, y1, y2 = self.crop_region
            depth_image = depth_image[x1 : shape[0] - x2, y1 : shape[1] - y2]

        mask = (depth_image < 0.2).astype(np.uint8)
        depth_image = cv2.inpaint(depth_image, mask, 3, cv2.INPAINT_NS)

        if hasattr(self, "blind_spot_crop"):
            shape = depth_image.shape
            x1, x2, y1, y2 = self.blind_spot_crop
            depth_image[:x1, :] = 0
            depth_image[shape[0] - x2 :, :] = 0
            depth_image[:, :y1] = 0
            depth_image[:, shape[1] - y2 :] = 0
        if hasattr(self, "gaussian_kernel_size"):
            depth_image = cv2.GaussianBlur(
                depth_image, self.gaussian_kernel_size, self.gaussian_sigma, self.gaussian_sigma
            )

        filt_m = np.clip(depth_image, self.depth_range[0], self.depth_range[1])
        filt_norm = (filt_m - self.depth_range[0]) / (self.depth_range[1] - self.depth_range[0])

        output_norm = filt_norm * (self.depth_output_range[1] - self.depth_output_range[0]) + self.depth_output_range[0]
        self.depth_image_buffer.append(output_norm)

    def _get_delayed_visualizable_image_obs(self):
        return self._get_depth_image_downsample_obs()

    def _get_depth_image_downsample_obs(self):
        self.refresh_depth_frame()
        return self.depth_image_buffer.buffer[self.depth_obs_indices, ...]

    def _vis_depth_obs(self, depth_obs: np.ndarray):
        depth_tiles = (np.clip(depth_obs, 0.0, 1.0) * 255).astype(np.uint8)
        rows, cols = 2, 4
        tile_h, tile_w = depth_tiles.shape[1], depth_tiles.shape[2]
        grid = np.zeros((rows * tile_h, cols * tile_w), dtype=np.uint8)
        for idx in range(depth_tiles.shape[0]):
            r, c = divmod(idx, cols)
            grid[r * tile_h : (r + 1) * tile_h, c * tile_w : (c + 1) * tile_w] = depth_tiles[idx]
        cv2.imwrite("depth_obs_grid.png", grid)


class ParkourStandAgent(ParkourAgent):
    def __init__(
        self,
        logdir: str,
        ros_node: RealNode,
    ):
        super().__init__(logdir, ros_node, depth_vis=False, pointcloud_vis=False)

    def _get_depth_image_downsample_obs(self):
        return np.zeros([len(self.depth_obs_indices), self.depth_height, self.depth_width])


class Body29DepthOn31Agent(ParkourStandAgent):
    """Run the original 29-DoF stand path on a 31-DoF onboard node."""

    BODY_DOF = 29

    def __init__(
        self,
        logdir: str,
        ros_node: RealNode,
        x_vel_scale: float = 0.5,
        y_vel_scale: float = 0.5,
        yaw_vel_scale: float = 1.0,
    ):
        self.x_vel_scale = x_vel_scale
        self.y_vel_scale = y_vel_scale
        self.yaw_vel_scale = yaw_vel_scale
        super().__init__(logdir=logdir, ros_node=ros_node)
        self._body_joint_ids = np.arange(self.BODY_DOF, dtype=np.int64)
        self._head_joint_ids = np.arange(self.BODY_DOF, self.ros_node.NUM_ACTIONS, dtype=np.int64)
        self._apply_head_defaults()

    def _parse_action_config(self):
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
                    if isinstance(actuator_config["stiffness"], dict):
                        for key, value in actuator_config["stiffness"].items():
                            if re.search(key, name):
                                self._p_gains[i] = value
                    else:
                        self._p_gains[i] = actuator_config["stiffness"]
                    if isinstance(actuator_config["damping"], dict):
                        for key, value in actuator_config["damping"].items():
                            if re.search(key, name):
                                self._d_gains[i] = value
                    else:
                        self._d_gains[i] = actuator_config["damping"]

        self._action_scale = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
        self._action_offset = self.default_joint_pos.copy()
        action_config = self.cfg["actions"]["joint_pos"]
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
        self._zero_action_joints = np.zeros(self.BODY_DOF, dtype=np.float32)

    def _apply_head_defaults(self):
        if self.ros_node.NUM_ACTIONS <= self.BODY_DOF:
            return
        head_default = robot_cfgs.G1_31Dof_TorsoBase.head_default_joint_pos
        self.default_joint_pos[self._head_joint_ids] = head_default[: len(self._head_joint_ids)]
        self._action_offset[self._head_joint_ids] = head_default[: len(self._head_joint_ids)]
        self._action_scale[self._head_joint_ids] = 0.0

    def _get_base_velocity_obs(self):
        x_vel = self.ros_node.joy_stick_data.ly * self.x_vel_scale
        y_vel = -self.ros_node.joy_stick_data.lx * self.y_vel_scale
        yaw_vel = -self.ros_node.joy_stick_data.rx * self.yaw_vel_scale
        self.xyyaw_command = np.array([x_vel, y_vel, yaw_vel], dtype=np.float32)
        return self.xyyaw_command

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

    def step(self):
        proprio_obs = []
        for proprio_obs_name in self.proprio_obs_names:
            obs_term_value = self._get_single_obs_term(proprio_obs_name)
            proprio_obs.append(np.reshape(obs_term_value, (1, -1)).astype(np.float32))
        proprio_obs = np.concatenate(proprio_obs, axis=-1)

        depth_obs = (
            self._get_single_obs_term(self.depth_obs_names[0])
            .reshape(1, -1, self.depth_height, self.depth_width)
            .astype(np.float32)
        )
        if hasattr(self.ros_node, "show_debug_depth_image"):
            self.ros_node.show_debug_depth_image(depth_obs[0, -1])
        depth_image_output = self.ort_sessions["depth_encoder"].run(
            None, {self.ort_sessions["depth_encoder"].get_inputs()[0].name: depth_obs}
        )[0]
        actor_input = np.concatenate([proprio_obs, depth_image_output], axis=1)
        actor_input_name = self.ort_sessions["actor"].get_inputs()[0].name
        body_action = self.ort_sessions["actor"].run(None, {actor_input_name: actor_input})[0].reshape(-1)

        full_action = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
        full_action[self._body_joint_ids] = body_action[: self.BODY_DOF]
        self._record_policy_io(
            "depth_encoder_actor_on31",
            {
                "proprio_obs": proprio_obs,
                "depth_obs": depth_obs,
                "depth_embedding": depth_image_output,
                "actor_input": actor_input,
                "body_action": body_action,
                "full_action": full_action,
                "target_joint_pos": full_action * self.action_scale + self.action_offset,
            },
        )
        return full_action, False
