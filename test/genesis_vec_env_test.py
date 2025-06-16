from dataclasses import dataclass
import json
from pathlib import Path
import shutil

import genesis as gs
from rich.pretty import pprint
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import VecNormalize
import torch.nn as nn
import tyro

from tag.gym.envs.walk.walk import Walk, WalkEnvConfig
from tag.gym.genesis_vec_env import GenesisVecEnv


def get_sb3_train_cfg(exp_name, total_timesteps):
    """Convert rsl_rl config to SB3 PPO config"""
    # Original rsl_rl config:
    # - learning_rate: 0.001
    # - gamma: 0.99
    # - clip_param: 0.2
    # - entropy_coef: 0.01
    # - value_loss_coef: 1.0
    # - max_grad_norm: 1.0
    # - num_learning_epochs: 5
    # - num_mini_batches: 4
    # - lam (GAE lambda): 0.95

    # JSON-serializable config for saving
    train_cfg_serializable = {
        "learning_rate": 0.001,
        "gamma": 0.99,
        "clip_range": 0.2,
        "ent_coef": 0.01,
        "vf_coef": 1.0,
        "max_grad_norm": 1.0,
        "n_epochs": 5,
        "batch_size": None,  # Will be calculated based on n_steps and n_minibatches
        "n_steps": 24,  # num_steps_per_env from original config
        "gae_lambda": 0.95,
        "policy_kwargs": {
            "activation_fn": "ELU",  # String representation for JSON
            "net_arch": {
                "pi": [512, 256, 128],  # actor_hidden_dims
                "vf": [512, 256, 128],  # critic_hidden_dims
            },
            "log_std_init": 0.0,  # Equivalent to init_noise_std=1.0 (log(1.0)=0.0)
        },
        "verbose": 1,
        "seed": 1,
        "device": "auto",
    }

    return train_cfg_serializable


def get_sb3_policy_kwargs():
    """Get the actual policy kwargs with non-serializable objects for model creation"""
    return {
        "activation_fn": nn.ELU,
        "net_arch": {
            "pi": [512, 256, 128],  # actor_hidden_dims
            "vf": [512, 256, 128],  # critic_hidden_dims
        },
        "log_std_init": 0.0,  # Equivalent to init_noise_std=1.0 (log(1.0)=0.0)
    }


def save_configs(log_dir, cfg, env_cfg, obs_cfg, train_cfg):
    if log_dir.exists():
        shutil.rmtree(log_dir)

    log_dir.mkdir(parents=True, exist_ok=True)

    with open(log_dir / "configs.json", "w") as f:
        json.dump(
            {
                "env_cfg": env_cfg,
                "obs_cfg": obs_cfg,
                "train_cfg": train_cfg,
            },
            f,
            indent=4,
        )


def get_cfgs():
    env_cfg = {
        "num_actions": 12,
        # joint/link names
        "default_joint_angles": {  # [rad]
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
        # PD
        "kp": 20.0,
        "kd": 0.5,
        # base pose
        # "base_init_pos": [0.0, 0.0, 0.42],
        # "base_init_quat": [1.0, 0.0, 0.0, 0.0],
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


@dataclass
class Config(WalkEnvConfig):
    exp_name: str = "go2-walking"
    train_steps: int = 101
    auto_reset: bool = True
    save_interval: int = 100  # Save model every N iterations
    use_vec_normalize: bool = True  # Enable observation normalization

    def __post_init__(self):
        self.train_steps += 1
        self.vis.pos = (2.0, 0.0, 2.5)


def main(cfg: Config):
    gs.init(logging_level="warning")
    pprint(cfg)

    env_cfg, obs_cfg = get_cfgs()

    # Create log directory
    log_dir = Path(f"logs/{cfg.exp_name}")
    log_dir.mkdir(parents=True, exist_ok=True)

    # Get number of environments from config
    n_envs = cfg.sim.num_envs

    # Calculate total timesteps
    # In rsl_rl: total_timesteps = train_steps * num_steps_per_env * num_envs
    num_steps_per_env = 24
    total_timesteps = cfg.train_steps * num_steps_per_env

    train_cfg = get_sb3_train_cfg(cfg.exp_name, total_timesteps)

    # Calculate batch_size for mini-batches
    # Original: num_mini_batches = 4
    # batch_size = (n_steps * n_envs) / num_mini_batches
    train_cfg["batch_size"] = (train_cfg["n_steps"] * n_envs) // 4

    # Save configurations
    save_configs(log_dir, cfg, env_cfg, obs_cfg, train_cfg)

    # Create environment
    env = Walk(
        cfg,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
    )
    env.build()
    vec_env = GenesisVecEnv(env)

    # Optional: Add observation normalization (equivalent to empirical_normalization in rsl_rl)
    if cfg.use_vec_normalize:
        vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True)

    # Create PPO model
    policy_kwargs = get_sb3_policy_kwargs()  # Get actual policy kwargs with nn.ELU
    model = PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=train_cfg["learning_rate"],
        gamma=train_cfg["gamma"],
        clip_range=train_cfg["clip_range"],
        ent_coef=train_cfg["ent_coef"],
        vf_coef=train_cfg["vf_coef"],
        max_grad_norm=train_cfg["max_grad_norm"],
        n_epochs=train_cfg["n_epochs"],
        batch_size=train_cfg["batch_size"],
        n_steps=train_cfg["n_steps"],
        gae_lambda=train_cfg["gae_lambda"],
        policy_kwargs=policy_kwargs,  # Use the actual policy kwargs
        verbose=train_cfg["verbose"],
        seed=train_cfg["seed"],
        device=train_cfg["device"],
        tensorboard_log=str(log_dir / "tensorboard"),
    )

    # Create callbacks
    callbacks = []

    # Checkpoint callback (equivalent to save_interval in rsl_rl)
    checkpoint_callback = CheckpointCallback(
        save_freq=cfg.save_interval * train_cfg["n_steps"],
        save_path=str(log_dir / "checkpoints"),
        name_prefix="ppo_walk",
        save_replay_buffer=False,
        save_vecnormalize=cfg.use_vec_normalize,
    )
    callbacks.append(checkpoint_callback)

    print(f"Starting training for {total_timesteps} timesteps...")
    print(f"Model will be saved every {cfg.save_interval} iterations to {log_dir}")

    # Train the model
    model.learn(
        total_timesteps=total_timesteps,
        callback=callbacks,
        tb_log_name=cfg.exp_name,
        progress_bar=True,
    )

    # Save final model
    model.save(str(log_dir / "final_model"))
    if cfg.use_vec_normalize:
        vec_env.save(str(log_dir / "vec_normalize.pkl"))

    print(f"Training completed! Model saved to {log_dir}")

    # Optional: Test the trained model
    print("Testing trained model...")
    obs = vec_env.reset()
    for i in range(100):
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, dones, info = vec_env.step(action)
        if dones.any():
            print(f"Episode finished at step {i}")
            obs = vec_env.reset()


if __name__ == "__main__":
    main(tyro.cli(Config))
