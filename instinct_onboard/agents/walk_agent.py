from __future__ import annotations

import os
import re

import numpy as np
import onnxruntime as ort

from instinct_onboard import robot_cfgs
from instinct_onboard.agents.base import OnboardAgent
from instinct_onboard.normalizer import Normalizer
from instinct_onboard.ros_nodes.base import RealNode


class WalkAgent(OnboardAgent):
    """A simple walk agent that uses only actor.onnx for continuous walking."""

    def __init__(
        self,
        logdir: str,
        ros_node: RealNode,
        x_vel_scale: float = 0.5,
        y_vel_scale: float = 0.5,
        yaw_vel_scale: float = 1.0,
    ):
        super().__init__(logdir, ros_node)
        self.ort_sessions = dict()
        self.x_vel_scale = x_vel_scale
        self.y_vel_scale = y_vel_scale
        self.yaw_vel_scale = yaw_vel_scale
        self._parse_obs_config()
        self._parse_action_config()
        self._load_models()

    def _load_models(self):
        """Load the ONNX model for the agent."""
        # load ONNX models
        actor_path = self._resolve_logdir_path("exported", "actor.onnx")
        self.ort_sessions["actor"] = ort.InferenceSession(actor_path, providers=["CPUExecutionProvider"])
        print(f"Loaded ONNX models from {self.logdir}")
        # optionally load the normalizer if it exists
        normalizer_path = self._resolve_logdir_path("exported", "policy_normalizer.npz")
        if os.path.exists(normalizer_path):
            self.normalizer = Normalizer(load_path=normalizer_path)
        else:
            self.normalizer = None

    def reset(self):
        """Reset the agent state."""
        super().reset()

    def step(self):
        """Perform a single step of the agent."""
        obs = self._get_observation()
        if self.normalizer is not None:
            normalized_obs = self.normalizer.normalize(obs).astype(np.float32)[None, :]
        else:
            normalized_obs = obs.astype(np.float32)[None, :]
        actor_input_name = self.ort_sessions["actor"].get_inputs()[0].name
        action = self.ort_sessions["actor"].run(None, {actor_input_name: normalized_obs})[0]
        action = action.reshape(-1)
        done = False  # Continuous walking, no termination
        return action, done

    """
    Agent specific observation functions for WalkAgent.
    """

    def _get_base_velocity_command_cmd_obs(self):
        """Return the base velocity command (from joystick)"""
        x_vel = self.ros_node.joy_stick_data.ly * self.x_vel_scale
        y_vel = -self.ros_node.joy_stick_data.lx * self.y_vel_scale
        yaw_vel = -self.ros_node.joy_stick_data.rx * self.yaw_vel_scale
        return np.array([x_vel, y_vel, yaw_vel])

    def _get_base_velocity_cmd_obs(self):
        """An alias for _get_base_velocity_command_obs"""
        return self._get_base_velocity_command_cmd_obs()


class Body29ActorOn31Agent(WalkAgent):
    """Run a 29-DoF actor-only policy on a 31-DoF onboard node.

    The first 29 joints are shared between the 29-DoF and 31-DoF robot definitions.
    Head joints are held at the 31-DoF default pose and excluded from the policy
    observation/action interface.
    """

    BODY_DOF = 29

    @staticmethod
    def _get_config_value_for_joint_or_default(value, joint_name: str, default):
        if isinstance(value, dict):
            for key, item in value.items():
                if re.search(key, joint_name):
                    return item
            return default
        return value

    def _get_body_joint_ids(self):
        return [self.ros_node.sim_joint_names.index(name) for name in robot_cfgs.G1_29Dof_TorsoBase.sim_joint_names]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._head_joint_ids = np.array(
            [
                self.ros_node.sim_joint_names.index("head_yaw_joint"),
                self.ros_node.sim_joint_names.index("head_pitch_joint"),
            ],
            dtype=np.int64,
        )
        self._body_joint_ids = np.array(
            self._get_body_joint_ids(),
            dtype=np.int64,
        )
        self._apply_head_defaults()

    def _parse_action_config(self):
        """Parse 29-DoF action config while leaving 31-DoF head joints fixed."""
        body_joint_ids = self._get_body_joint_ids()
        self.default_joint_pos = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
        for joint_name_expr, joint_pos in self.cfg["scene"]["robot"]["init_state"]["joint_pos"].items():
            for i in body_joint_ids:
                name = self.ros_node.sim_joint_names[i]
                if re.search(joint_name_expr, name):
                    self.default_joint_pos[i] = joint_pos

        self.default_joint_vel = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
        for joint_name_expr, joint_vel in self.cfg["scene"]["robot"]["init_state"]["joint_vel"].items():
            for i in body_joint_ids:
                name = self.ros_node.sim_joint_names[i]
                if re.search(joint_name_expr, name):
                    self.default_joint_vel[i] = joint_vel

        self._p_gains = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
        self._d_gains = np.zeros(self.ros_node.NUM_JOINTS, dtype=np.float32)
        for actuator_config in self.cfg["scene"]["robot"]["actuators"].values():
            for i in body_joint_ids:
                name = self.ros_node.sim_joint_names[i]
                for joint_name_expr in actuator_config["joint_names_expr"]:
                    if not re.search(joint_name_expr, name):
                        continue
                    self._p_gains[i] = self._get_config_value_for_joint_or_default(
                        actuator_config["stiffness"], name, 0.0
                    )
                    self._d_gains[i] = self._get_config_value_for_joint_or_default(
                        actuator_config["damping"], name, 0.0
                    )

        self._action_scale = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
        self._action_offset = self.default_joint_pos.copy()
        action_config = self.cfg["actions"]["joint_pos"]
        use_default_offset = action_config.get("use_default_offset", True)
        offset = action_config.get("offset", 0.0)
        scale = action_config.get("scale", 1.0)
        for i in body_joint_ids:
            name = self.ros_node.sim_joint_names[i]
            for joint_name_expr in action_config["joint_names"]:
                if not re.search(joint_name_expr, name):
                    continue
                self._action_scale[i] = self._get_config_value_for_joint_or_default(scale, name, 1.0)
                if not use_default_offset:
                    self._action_offset[i] = self._get_config_value_for_joint_or_default(offset, name, 0.0)
                break

    def _apply_head_defaults(self):
        if self.ros_node.NUM_ACTIONS <= self.BODY_DOF:
            return
        head_default = robot_cfgs.G1_31Dof_TorsoBase.head_default_joint_pos
        self.default_joint_pos[self._head_joint_ids] = head_default[: len(self._head_joint_ids)]
        self._action_offset[self._head_joint_ids] = head_default[: len(self._head_joint_ids)]
        self._action_scale[self._head_joint_ids] = 0.0

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
        body_action, done = super().step()
        if self.ros_node.NUM_ACTIONS <= self.BODY_DOF:
            return body_action, done
        full_action = np.zeros(self.ros_node.NUM_ACTIONS, dtype=np.float32)
        full_action[self._body_joint_ids] = body_action[: self.BODY_DOF]
        return full_action, done


class Body29StandOn31Agent(Body29ActorOn31Agent):
    """Run a 29-DoF stand policy on a 31-DoF node with zero velocity command."""

    def _get_base_velocity_command_cmd_obs(self):
        return np.zeros(3, dtype=np.float32)

    def _get_base_velocity_cmd_obs(self):
        return self._get_base_velocity_command_cmd_obs()
