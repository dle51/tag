from dataclasses import dataclass
import json
from pathlib import Path
import shutil

from stable_baselines3 import PPO
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import SubprocVecEnv
import tyro

from tag.gym.envs.walk.walk import Walk, WalkEnvConfig


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
        "num_obs": 60,
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
        },
    }

    return env_cfg, obs_cfg


def save_configs(log_dir, cfg, env_cfg, obs_cfg):
    if log_dir.exists():
        shutil.rmtree(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    with open(log_dir / "configs.json", "w") as f:
        json.dump(
            {
                "cfg": vars(cfg),
                "env_cfg": env_cfg,
                "obs_cfg": obs_cfg,
            },
            f,
            indent=4,
        )


@dataclass
class Config(WalkEnvConfig):
    exp_name: str = "go2-walking"
    train_steps: int = 1_000_000
    auto_reset: bool = True

    def __post_init__(self):
        self.vis.pos = (2.0, 0.0, 2.5)


def make_env(rank):
    def _init():
        env = Walk(cfg, env_cfg=env_cfg, obs_cfg=obs_cfg)
        env.build()
        return env

    return _init


def main(cfg: Config):
    env_cfg, obs_cfg = get_cfgs()
    log_dir = Path(f"logs/{cfg.exp_name}")
    save_configs(log_dir, cfg, env_cfg, obs_cfg)

    # Use e.g., 16 parallel envs (adjust to match your CPU cores)
    num_envs = 16
    env = SubprocVecEnv([make_env(i) for i in range(num_envs)])

    # SB3 logger
    new_logger = configure(log_dir.as_posix(), ["stdout", "csv", "tensorboard"])

    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        learning_rate=1e-3,
        n_steps=24,  # match rsl_rl num_steps_per_env
        batch_size=96,  # 24 * 4 mini-batches from rsl config
        n_epochs=5,
        clip_range=0.2,
        ent_coef=0.01,
        gae_lambda=0.95,
        gamma=0.99,
        max_grad_norm=1.0,
    )
    model.set_logger(new_logger)

    model.learn(total_timesteps=cfg.train_steps)
    model.save(log_dir / "model")


if __name__ == "__main__":
    main(tyro.cli(Config))
