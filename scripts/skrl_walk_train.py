from dataclasses import dataclass

import genesis as gs
from skrl.agents.torch.ppo import PPO
from skrl.envs.wrappers.torch import wrap_env
from skrl.memories.torch import RandomMemory
from skrl.models.torch import DeterministicMixin, GaussianMixin, Model
from skrl.resources.preprocessors.torch import RunningStandardScaler
from skrl.resources.schedulers.torch import KLAdaptiveRL
from skrl.trainers.torch import SequentialTrainer
import torch
import torch.nn as nn
import tyro

from tag.gym.envs.walk.walk import GymWrapper, Walk, WalkEnvConfig


def get_train_cfgs(exp_name, env):
    # TODO: Fix Parameters
    train_cfg = {
        "rollouts": 24,
        "learning_epochs": 5,
        "mini_batches": 4,
        "discount_factor": 0.99,
        "lambda": 0.95,
        "learning_rate": 0.001,
        "learning_rate_scheduler": KLAdaptiveRL,
        "learning_rate_scheduler_kwargs": {"kl_threshold": 0.01},
        "state_preprocessor": RunningStandardScaler,
        "state_preprocessor_kwargs": {"size": env.observation_space, "device": env.device},
        "value_preprocessor": RunningStandardScaler,
        "value_preprocessor_kwargs": {"size": 1},
        "random_timesteps": 0,
        "learning_starts": 0,
        "grad_norm_clip": 0.5,
        "ratio_clip": 0.2,
        "value_clip": 0.2,
        "clip_predicted_values": True,
        "entropy_loss_scale": 0.01,
        "value_loss_scale": 1.0,
        "kl_threshold": 0,
        "rewards_shaper": None,
        "time_limit_bootstrap": False,
        "mixed_precision": False,
        "experiment": {
            "directory": "logs/",
            "experiment_name": exp_name,
            "write_interval": "auto",
            "checkpoint_interval": 100,
            "store_separately": False,
            "wandb": True,
            "wandb_kwargs": {"project": "walk", "name": exp_name},
        },
    }

    return train_cfg


def get_cfgs():
    env_cfg = {
        "num_actions": 12,
        "default_joint_angles": {
            "FL_hip_joint": 0.0,
            "FR_hip_joint": 0.0,
            "RL_hip_joint": 0.0,
            "RR_hip_joint": 0.0,
            "FL_thigh_joint": 0.8,
            "FR_thigh_joint": 0.8,
            "RL_thigh_joint": 1.0,
            "RR_thigh_joint": 1.0,
            "FL_calf_joint": -1.5,
            "FR_calf_joint": -1.5,
            "RL_calf_joint": -1.5,
            "RR_calf_joint": -1.5,
        },
        "kp": 20.0,
        "kd": 0.5,
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.25,
        "simulate_action_latency": True,
        "clip_actions": 100.0,
    }

    obs_cfg = {
        "num_obs": 48,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    return env_cfg, obs_cfg


class Policy(GaussianMixin, Model):
    def __init__(
        self,
        observation_space,
        action_space,
        device,
        clip_actions=False,
        clip_log_std=-20,
        min_log_std=2,
        max_log_std=2,
        reduction="sum",
    ):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(self, clip_actions, clip_log_std, min_log_std, max_log_std, reduction)

        self.net = nn.Sequential(
            nn.Linear(self.num_observations, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, self.num_actions),
        )
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions))

    def compute(self, inputs, role):
        return self.net(inputs["states"]), self.log_std_parameter, {}


class Value(DeterministicMixin, Model):
    def __init__(self, observation_space, action_space, device, clip_actions=False):
        Model.__init__(self, observation_space, action_space, device)
        DeterministicMixin.__init__(self, clip_actions)

        self.net = nn.Sequential(
            nn.Linear(self.num_observations, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 1),
        )

    def compute(self, inputs, role):
        return self.net(inputs["states"]), {}


@dataclass
class Config(WalkEnvConfig):
    exp_name: str = "skrl-walking"
    train_steps: int = 101
    auto_reset: bool = True

    def __post_init__(self):
        # super().__post_init__()
        self.train_steps += 1
        self.vis.pos = (2.0, 0.0, 2.5)


def main(cfg: Config):
    gs.init(logging_level="warning")

    env_cfg, obs_cfg = get_cfgs()
    genesis_env = Walk(cfg, env_cfg=env_cfg, obs_cfg=obs_cfg)
    genesis_env.build()

    # Testing Gym Wrapper
    gym_env = GymWrapper(genesis_env)

    # SKRL Wrapping
    env = wrap_env(gym_env, wrapper="gymnasium")

    # Train Config
    train_cfg = get_train_cfgs(cfg.exp_name, env)

    # Memory Instantiatation
    memory = RandomMemory(memory_size=1024, num_envs=env.num_envs, device=env.device)

    # Models
    models = {}
    models["policy"] = Policy(env.observation_space, env.action_space, device=env.device, clip_actions=True)
    models["value"] = Value(env.observation_space, env.action_space, device=env.device)

    # TODO(dle): Implement Gaussian Noise

    # PPO Agent
    agent = PPO(
        models=models,
        memory=memory,
        cfg=train_cfg,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device,
    )

    # Configure Trainer
    trainer_cfg = {"timesteps": (env.num_envs * cfg.train_steps), "headless": True}

    # Trainer
    trainer = SequentialTrainer(cfg=trainer_cfg, env=env, agents=[agent])

    # Train
    trainer.train()


if __name__ == "__main__":
    main(tyro.cli(Config))
