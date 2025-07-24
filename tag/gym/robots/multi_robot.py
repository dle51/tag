from copy import deepcopy
from dataclasses import dataclass

import genesis as gs
import numpy as np
import torch

from tag.gym.robots.robot import Robot, RobotConfig
from tag.utils import default

ListLike = list | tuple | torch.Tensor | np.ndarray


@dataclass
class MultiRobotConfig:
    cfgs: list[RobotConfig] = default([])


class MultiRobot(Robot):
    """Version 2"""

    def __init__(self, scene: gs.Scene, cfg: MultiRobotConfig, n: list[str] | int, colors=None):
        self.cfg = cfg
        self._init_robots(scene, n, colors)

    def _init_robots(self, scene: gs.Scene, n: list[str] | int, colors=None):
        self.robots = {}
        num_robots = n if isinstance(n, int) else len(n)
        for i in range(num_robots):
            cfg = deepcopy(self.cfg.cfgs[i])
            # TODO(dle): Implement DR - Random spawn locations
            # TODO(dle): Implement Surface Colors
            robot: Robot = cfg.create(scene)
            name = ("robot_" + str(i)) if isinstance(n, int) else n[i]
            self.robots[name] = robot

    # RL Interface Methods

    def act(self, actions: ListLike, mode: str = "position"):
        for k, robot in self.robots.values():
            robot.act(actions[k])

    def observe(self):
        return {k: robot.observe() for k, robot in self}

    def reset(self, envs_idx: ListLike):
        """
        Reset all robots in specified parallel environments.
        """
        # TODO(dle): Implement DR - Random spawn locations
        for robot in self.robots.values():
            robot.reset(envs_idx)

    # Metadata Methods

    @property
    def observation_space(self):
        pass

    @property
    def single_observation_space(self):
        pass

    @property
    def action_space(self):
        pass

    @property
    def single_action_space(self):
        pass

    # Protocol Methods
    def __iter__(self):
        return iter(self.robots.values())

    def wrapped(self):
        raise NotImplementedError("MultiRobot cannot be unwrapped.")
