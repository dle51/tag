from dataclasses import dataclass
import math
from typing import Any, Tuple

import genesis as gs
from genesis.utils.geom import inv_quat, quat_to_xyz, transform_by_quat, transform_quat_by_quat
import gymnasium as gym
import numpy as np
import torch

from tag.gym.envs.mixins.reward import ChaseReward, RewardMixin
from tag.gym.envs.robotic import MultiGo2EnvConfig, MultiRobotEnv
from tag.gym.robots.joystick_go2 import OVERFIT, CommandConfig
from tag.utils import default, defaultcls

ListLike = list | tuple | torch.Tensor | np.ndarray


def _rand_float(lower, upper, shape, device):
    return (upper - lower) * torch.rand(size=shape, device=device) + lower


def _float(shape):
    return torch.zeros(shape, device=gs.device, dtype=gs.tc_float)


def _int(shape):
    return torch.zeros(shape, device=gs.device, dtype=gs.tc_int)


@dataclass
class ChaseEnvConfig(MultiGo2EnvConfig):
    # TODO(dle): Implement commands, scales, rewards, Etc.
    command: CommandConfig = default(OVERFIT)  # Placeholder
    rewards: ChaseReward = defaultcls(ChaseReward)  # Needs to be implemented
    auto_reset: bool = True


class SingleGymWrapper(gym.vector.VectorEnv):
    """
    A Gymnasium VectorEnv wrapper for a multi-robotic environment.
    Allows for the Policy to send actions to a singular (first) robot.
    Minimum inplementation for compatibility with the SKRL Library.
    """

    def __init__(self, env):
        self.env = env
        self.num_envs = env.num_envs

        self.observation_space = gym.spaces.Box(low=-10.0, high=10.0, shape=(self.num_envs, 96), dtype=np.float32)
        self.single_observation_space = gym.spaces.Box(low=-10.0, high=10.0, shape=(96,), dtype=np.float32)

        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(self.num_envs, 12), dtype=np.float32
        )  # Set to 1 due for tanh
        self.single_action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(12,), dtype=np.float32)

        self.metadata = {"autoreset_mode": "disabled"}

        self.timestep = 0

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        """
        Resets all parallel environments and returns batched observations and info.
        """
        obs_buf, _ = self.env.reset()
        return obs_buf, {}

    def step(self, actions) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, dict[str, Any]]:
        """
        Take an action for each parallel environment.
        Returns batch of observations, rewards, terminations, truncations, and info.
        """
        self.timestep += 1
        obs_buf, rew_buf, terminated, truncated, infos = self.env.step(actions, selection=0)

        return obs_buf, rew_buf, terminated, truncated, infos


class Chase(MultiRobotEnv, RewardMixin):
    """Version 2"""

    def __init__(self, cfg: ChaseEnvConfig, env_cfg, obs_cfg):
        super().__init__(cfg)
        self.cfg = cfg

        if self.cfg.cam.follow:
            self.cam_follow(self.robot.robot)

        self.num_envs = self.cfg.sim.num_envs
        self.num_obs = 96  # 48 observations per Go2 robot
        self.num_actions = env_cfg["num_actions"]  # 12 actions per Go2 robot
        self.resample_t = int(env_cfg["resampling_time_s"] / self.cfg.sim.dt)

        self.simulate_action_latency = True  # TODO: use cfg
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.cfg.sim.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.obs_scales = obs_cfg["obs_scales"]

        # Robot Params
        self.robot_params = {}

        self._init_robot_params()
        self._init_buffers()

    def build(self):
        super().build()

    def _resample_commands(self, envs_idx):
        # NOTE(dle): explain?
        for robot in self.robots.robots.keys():
            self.robot_buf[robot]["commands"][envs_idx, 0] = _rand_float(
                *self.cfg.command.lin_vel_x_range, (len(envs_idx),), gs.device
            )
            self.robot_buf[robot]["commands"][envs_idx, 1] = _rand_float(
                *self.cfg.command.lin_vel_y_range, (len(envs_idx),), gs.device
            )
            self.robot_buf[robot]["commands"][envs_idx, 2] = _rand_float(
                *self.cfg.command.ang_vel_range, (len(envs_idx),), gs.device
            )

    def step(self, actions, actions_2=None, selection: int = 0):
        """
        The 'selection' argument determines which robot `actions` passes to:
            selection = 0: Robot 1
            selection = 1: Robot 2
            selection = 2: Robots 1 & 2 with actions & actions_2
        """

        assert selection == 0 or selection == 1 or selection == 2, "Invalid selection in step method."

        exec_actions = {}
        target_dof_pos = {}
        robot_1, robot_2 = list(self.robots.robots.keys())[0], list(self.robots.robots.keys())[1]

        # Actions update
        if selection == 0:
            self.robot_buf[robot_1]["actions"] = torch.tensor(actions, dtype=torch.float32, device=self.device)
        elif selection == 1:
            self.robot_buf[robot_2]["actions"] = torch.tensor(actions, dtype=torch.float32, device=self.device)
        elif selection == 2:
            self.robot_buf[robot_1]["actions"] = torch.tensor(actions, dtype=torch.float32, device=self.device)
            self.robot_buf[robot_2]["actions"] = torch.tensor(actions_2, dtype=torch.float32, device=self.device)

        # exec_actions
        if selection == 0 or selection == 2:
            exec_actions[robot_1] = (
                self.robot_buf[robot_1]["last_actions"]
                if self.simulate_action_latency
                else self.robot_buf[robot_1]["actions"]
            )
        if selection == 1 or selection == 2:
            exec_actions[robot_2] = (
                self.robot_buf[robot_2]["last_actions"]
                if self.simulate_action_latency
                else self.robot_buf[robot_2]["actions"]
            )

        # target_dof_pos
        if selection == 0 or selection == 2:
            target_dof_pos[robot_1] = (
                exec_actions[robot_1] * self.env_cfg["action_scale"] + self.robot_buf[robot_1]["default_dof_pos"]
            )
        if selection == 1 or selection == 2:
            target_dof_pos[robot_2] = (
                exec_actions[robot_2] * self.env_cfg["action_scale"] + self.robot_buf[robot_2]["default_dof_pos"]
            )

        # Genesis movement
        if selection == 0 or selection == 2:
            self.robots.robots[robot_1].control_dofs_position(target_dof_pos[robot_1], self.robots.robots[robot_1].dofs)
        if selection == 1 or selection == 2:
            self.robots.robots[robot_2].control_dofs_position(target_dof_pos[robot_2], self.robots.robots[robot_2].dofs)

        self.scene.step()

        # Updating buffers
        self.episode_length_buf += 1
        inv_base_quat = {}
        base_euler = {}
        for robot in self.robot_buf.keys():
            self.robot_buf[robot]["base_pos"][:] = self.robots.robots[robot].pos
            self.robot_buf[robot]["base_quat"][:] = self.robots.robots[robot].quat

            inv_base_quat[robot] = inv_quat(self.robot_buf[robot]["base_quat"])
            base_euler[robot] = quat_to_xyz(
                transform_quat_by_quat(
                    torch.ones_like(self.robot_buf[robot]["base_quat"])
                    * self.robot_params[robot]["inv_base_init_quat"],
                    self.robot_buf[robot]["base_quat"],
                ),
                rpy=True,
                degrees=False,
            )

            self.robot_buf[robot]["base_lin_vel"] = transform_by_quat(
                self.robots.robots[robot].get_vel(), inv_base_quat[robot]
            )
            self.robot_buf[robot]["base_ang_vel"] = transform_by_quat(
                self.robots.robots[robot].get_ang(), inv_base_quat[robot]
            )
            self.robot_buf[robot]["projected_gravity"] = transform_by_quat(self.global_gravity, inv_base_quat[robot])
            self.robot_buf[robot]["dof_pos"] = self.robots.robots[robot].get_dofs_position(
                self.robots.robots[robot].dofs
            )
            self.robot_buf[robot]["dof_vel"] = self.robots.robots[robot].get_dofs_velocity(
                self.robots.robots[robot].dofs
            )

        # Check resampling
        envs_idx = self.episode_length_buf % self.resample_t == 0
        envs_idx = envs_idx.nonzero(as_tuple=False).flatten()
        self._resample_commands(envs_idx)

        # Check termination & truncation
        # TODO(dle): Implement out of bounds termination
        self.checks = {
            "truncate": self.episode_length_buf > self.max_episode_length,
            "pitch": torch.abs(base_euler[robot_1][:, 1]) > self.cfg.rewards.termination_if_pitch_greater_than
            or torch.abs(base_euler[robot_2][:, 1]) > self.cfg.rewards.termination_if_pitch_greater_than,
            "roll": torch.abs(base_euler[robot_1][:, 0]) > self.cfg.rewards.termination_if_roll_greater_than
            or torch.abs(base_euler[robot_2][:, 0]) > self.cfg.rewards.termination_if_roll_greater_than,
            "height": self.robot_buf[robot_1]["base_pos"][:, 2] < self.cfg.rewards.termination_if_height_lower_than
            or self.robot_buf[robot_2]["base_pos"][:, 2] < self.cfg.rewards.termination_if_height_lower_than,
        }
        # TODO/NOTE(dle): Where to implement resetting without termination/truncation - completion upon collision
        self.reset_buf = self.checks["truncate"] | self.checks["pitch"] | self.checks["roll"] | self.checks["height"]
        truncated = self.checks["truncate"]
        terminated = self.checks["pitch"] | self.checks["roll"] | self.checks["height"]

        if self.cfg.auto_reset:
            self.reset_idx(self.reset_buf.nonzero(as_tuple=False).flatten())

        # self.compute_reward() - Not Implemented
        self.render()

        # Computing observations
        robot_1_buf = torch.cat(
            [
                self.robot_buf[robot_1]["base_lin_vel"] * self.obs_scales["lin_vel"],
                self.robot_buf[robot_1]["base_ang_vel"] * self.obs_scales["ang_vel"],
                self.robot_buf[robot_1]["projected_gravity"],
                self.robot_buf[robot_1]["commands"] * self.commands_scale,
                (self.robot_buf[robot_1]["dof_pos"] - self.robot_buf[robot_1]["default_dof_pos"])
                * self.obs_scales["dof_pos"],
                self.robot_buf[robot_1]["dof_vel"] * self.obs_scales["dof_vel"],
                self.robot_buf[robot_1]["actions"] * self.env_cfg["action_scale"],
            ],
            axis=-1,
        )
        robot_2_buf = torch.cat(
            [
                self.robot_buf[robot_2]["base_lin_vel"] * self.obs_scales["lin_vel"],
                self.robot_buf[robot_2]["base_ang_vel"] * self.obs_scales["ang_vel"],
                self.robot_buf[robot_2]["projected_gravity"],
                self.robot_buf[robot_2]["commands"] * self.commands_scale,
                (self.robot_buf[robot_2]["dof_pos"] - self.robot_buf[robot_2]["default_dof_pos"])
                * self.obs_scales["dof_pos"],
                self.robot_buf[robot_2]["dof_vel"] * self.obs_scales["dof_vel"],
                self.robot_buf[robot_2]["actions"] * self.env_cfg["action_scale"],
            ],
            axis=-1,
        )

        self.obs_buf = torch.hstack((robot_1_buf, robot_2_buf))

        for robot in self.robots.robots.keys():
            self.robot_buf[robot]["last_actions"][:] = self.robot_buf[robot]["actions"][:]
            self.robot_buf[robot]["last_dof_vel"][:] = self.robot_buf[robot]["dof_vel"][:]

        self.extras["observations"]["critic"] = self.obs_buf

        return self.obs_buf, self.rew_buf, terminated, truncated, self.extras

    def reset_idx(self, envs_idx: ListLike = None):
        """Resets specified environments to default states."""

        # Reset all indices
        if envs_idx is None:
            envs_idx = [i for i in range(self.num_envs)]

        # No resets
        if len(envs_idx) == 0:
            return

        # Resetting dofs and base
        for robot in self.robot_buf.keys():
            self.robot_buf[robot]["dof_pos"][envs_idx] = self.robot_buf[robot]["default_dof_pos"]
            self.robot_buf[robot]["dof_vel"][envs_idx] = 0.0

            self.robot_buf[robot]["base_pos"][envs_idx] = self.robot_params[robot]["base_init_pos"][envs_idx].clone()
            self.robot_buf[robot]["base_quat"][envs_idx] = self.robot_params[robot]["base_init_quat"][envs_idx].clone()

            self.robot_buf[robot]["base_lin_vel"][envs_idx] = 0
            self.robot_buf[robot]["base_ang_vel"][envs_idx] = 0

        # Resetting model in Genesis
        for robot in self.robots.robots.keys():
            # dofs
            self.robots.robots[robot].set_dofs_position(
                position=self.robot_buf[robot]["dof_pos"][envs_idx],
                dofs_idx_local=self.robots.robots[robot].dofs,
                zero_velocity=True,
                envs_idx=envs_idx,
            )
            # base
            self.robots.robots[robot].set_pos(
                self.robot_buf[robot]["base_pos"][envs_idx], zero_velocity=False, envs_idx=envs_idx
            )
            self.robots.robots[robot].set_quat(
                self.robot_buf[robot]["base_quat"][envs_idx], zero_velocity=False, envs_idx=envs_idx
            )
            self.robots.robots[robot].zero_all_dofs_velocity(envs_idx)

        # Resetting buffers
        for robot in self.robots.robots.keys():
            self.robot_buf[robot]["last_actions"][envs_idx] = 0.0
            self.robot_buf[robot]["last_dof_vel"][envs_idx] = 0.0
        self.episode_length_buf[envs_idx] = 0
        self.reset_buf[envs_idx] = 0.0

        # extras
        self.extras["episode"] = {}

        self._resample_commands(envs_idx)

    def reset(self):
        """Resets all parallel environments."""
        self.reset_idx()

        return self.obs_buf, None

    def _init_buffers(self):
        # Interface Buffers
        self.obs_buf = _float((self.num_envs, self.num_obs))
        self.rew_buf = _float((self.num_envs,))
        self.reset_buf = _int((self.num_envs,))
        self.episode_length_buf = _int((self.num_envs,))

        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], device=gs.device, dtype=gs.tc_float).repeat(
            self.num_envs, 1
        )
        self.extras = dict()
        self.extras["observations"] = dict()

        self.commands_scale = torch.tensor(
            [
                self.obs_scales["lin_vel"],
                self.obs_scales["lin_vel"],
                self.obs_scales["ang_vel"],
            ],
            device=gs.device,
            dtype=gs.tc_float,
        )

        # Robot Buffers
        self.robot_buf = {}
        for robot in self.robots.robots.keys():
            self.robot_buf[robot] = {}

            self.robot_buf[robot]["base_lin_vel"] = _float((self.num_envs, 3))
            self.robot_buf[robot]["base_ang_vel"] = _float((self.num_envs, 3))

            self.robot_buf[robot]["projected_gravity"] = _float((self.num_envs, 3))

            self.robot_buf[robot]["default_dof_pos"] = torch.tensor(
                [self.robots.robots[robot].cfg.state.joints[name] for name in self.robots.robots[robot].cfg.dof_names],
                device=gs.device,
                dtype=gs.tc_float,
            )

            self.robot_buf[robot]["actions"] = _float((self.num_envs, 12))
            self.robot_buf[robot]["last_actions"] = torch.zeros_like(self.robot_buf[robot]["actions"])
            self.robot_buf[robot]["dof_pos"] = torch.zeros_like(self.robot_buf[robot]["actions"])
            self.robot_buf[robot]["dof_vel"] = torch.zeros_like(self.robot_buf[robot]["actions"])
            self.robot_buf[robot]["last_dof_vel"] = torch.zeros_like(self.robot_buf[robot]["actions"])
            self.robot_buf[robot]["base_pos"] = _float((self.num_envs, 3))
            self.robot_buf[robot]["base_quat"] = _float((self.num_envs, 4))

            self.robot_buf[robot]["commands"] = _float((self.num_envs, self.cfg.command.num_commands))

    def _init_robot_params(self):
        for robot in self.robots.robots.keys():
            self.robot_params[robot] = {}

            self.robot_params[robot]["base_init_pos"] = torch.tensor(
                self.robots.robots[robot].cfg.state.pos, device=gs.device
            )
            self.robot_params[robot]["base_init_quat"] = torch.tensor(
                self.robots.robots[robot].cfg.state.quat, device=gs.device
            )
            self.robot_params[robot]["inv_base_init_quat"] = inv_quat(self.robot_params[robot]["base_init_quat"])
