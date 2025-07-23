import genesis as gs
import numpy as np
import torch
import math

from dataclasses import dataclass
from genesis.utils.geom import inv_quat, quat_to_xyz, transform_by_quat, transform_quat_by_quat
from tag.gym.robots.multi_robot import MultiRobot, MultiRobotConfig
from tag.gym.envs.robotic import MultiGo2EnvConfig, MultiRobotEnv

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
    auto_reset: bool = True

class Chase(MultiRobotEnv):
    """Version 2"""
    def __init__(self, cfg: ChaseEnvConfig, env_cfg, obs_cfg):
        super().__init__(cfg)
        self.cfg = cfg
        
        if self.cfg.cam.follow:
            self.cam_follow(self.robot.robot)

        self.num_envs = self.cfg.sim.num_envs
        self.num_obs = 96 # 48 observations per Go2 robot
        self.num_actions = env_cfg["num_actions"] # 12 actions per Go2 robot
        self.resample_t = int(env_cfg["resampling_time_s"] / self.cfg.sim.dt)

        self.simulate_action_latency = True # TODO: use cfg
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.cfg.sim.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg

        # Robot Params
        self.base_init_pos = []
        self.base_init_quat = []
        self.inv_base_init_quat = []

        self._init_robot_params()


    def build(self):
        super().build()
        
    def reset_idx(self, envs_idx: ListLike | bool = True):
        """Resets specified environments to default states."""

        # Resetting in Genesis
        cfg_num = 0


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

        # Robot Buffers


    def _init_robot_params(self):
        for cfg in self.cfg.cfgs:
            self.base_init_pos.append(torch.tensor(cfg.robot.state.pos, device=gs.device))
            self.base_init_quat.append(torch.tensor(cfg.robot.state.quat, device=gs.device))
            self.inv_base_init_quat.append(inv_quat(self.base_init_quat[-1]))
