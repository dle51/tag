from dataclasses import dataclass
import json
from pathlib import Path
import random

import genesis as gs
from go2_train import Config
from rsl_rl.runners import OnPolicyRunner
import torch
from tqdm import tqdm
import tyro

from tag.gym.envs.walk.walk import Walk
from tag.gym.robots.joystick_go2 import (
    CommandConfig,
    commands,
)
from tag.names import BASE


def run_super_fast() -> CommandConfig:
    return CommandConfig(
        num_commands=3, lin_vel_x_range=[4.0, 4.0], lin_vel_y_range=[0.0, 0.0], ang_vel_range=[0.0, 0.0]
    )


def random_command_config() -> CommandConfig:
    # Define valid limits
    lin_vel_x_min, lin_vel_x_max = -1.75, 2.25
    lin_vel_y_min, lin_vel_y_max = -1.75, 1.75
    ang_vel_min, ang_vel_max = -1.0, 1.0

    # Sample a fixed value in range for each dimension
    lin_vel_x = random.uniform(lin_vel_x_min, lin_vel_x_max)
    lin_vel_y = random.uniform(lin_vel_y_min, lin_vel_y_max)
    ang_vel = random.uniform(ang_vel_min, ang_vel_max)

    # Create config with fixed value ranges
    return CommandConfig(
        num_commands=3,
        lin_vel_x_range=[lin_vel_x, lin_vel_x],
        lin_vel_y_range=[lin_vel_y, lin_vel_y],
        ang_vel_range=[ang_vel, ang_vel],
    )


def load_configs(log_dir):
    with open(log_dir / "configs.json", "r") as f:
        _cfgs = json.load(f)

    env_cfg, obs_cfg, train_cfg = (
        _cfgs["env_cfg"],
        _cfgs["obs_cfg"],
        # _cfgs["command_cfg"],
        _cfgs["train_cfg"],
    )
    return env_cfg, obs_cfg, train_cfg


@dataclass
class EvalConfig(Config):
    ckpt: int = 100  # the checkpoint to load
    auto_reset: bool = False
    use_ts: bool = False
    path: str | Path = None


def main(cfg: EvalConfig):
    # check_rsl_rl()
    gs.init(logging_level="critical")

    if not cfg.use_ts:
        log_dir = Path(cfg.path)
        env_cfg, obs_cfg, train_cfg = load_configs(log_dir)
    c = "FF"
    cfg.command = commands[c]
    env = Walk(
        cfg,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
    )

    if cfg.use_ts:
        pi = torch.jit.load(str(BASE / "joy.ts.pt")).to(gs.device)
    else:
        runner = OnPolicyRunner(env, train_cfg, log_dir, device=gs.device)
        resume_path = log_dir / f"model_{cfg.ckpt}.pt"
        runner.load(str(resume_path))
        pi = runner.get_inference_policy(device=gs.device)

    # Speed Eval
    env.build()
    env.cam.follow_entity(env.robot.robot)
    env.cfg.command = run_super_fast()
    obs, _ = env.reset()
    with torch.no_grad():
        for i in tqdm(range(len(env)), desc="Eval..."):
            actions = pi(obs)
            obs, rews, dones, infos = env.step(actions)
    name = "RunSuperFast_eval"

    env.record_visualization(name)
    print("File saved to " + name)
    """
    # Basic Eval - Forward+Backwards+Sideways+Turn
    env.build()
    env.cam.follow_entity(env.robot.robot)
    cc = ["FF", "BF", "SRF", "SLF", "FTRF", "FTLF", "BTRF", "BTLF"]
    for key in cc:
        if env.cam._in_recording is False:
            env.cam.start_recording()
        env.cfg.command = commands[key]
        obs, _ = env.reset()
        with torch.no_grad():
            for i in tqdm(range(len(env)), desc="Eval..."):
                actions = pi(obs)
                obs, rews, dones, infos = env.step(actions)

        name = cfg.exp_name + "_" + str(cfg.ckpt) + "_" + key + "_eval"

        env.record_visualization(name)
        print("File saved to " + name)
    """
    """
    env.build()
    env.cam.follow_entity(env.robot.robot)
    for c in range(10):
        if env.cam._in_recording is False:
            env.cam.start_recording()
        env.cfg.command = commands["FF"]
        obs, _ = env.reset()
        with torch.no_grad():
            for i in tqdm(range(len(env)), desc="Eval..."):
                if i % 250 == 0:
                    choice = random.choice(list(commands.values()))
                    env.cfg.command = choice
                    env.resam_commands()
                actions = pi(obs)
                obs, rews, dones, infos = env.step(actions)

        name = cfg.exp_name + "_" + str(cfg.ckpt) + "_" + str(c) + "_eval"

        env.record_visualization(name)
        print("File saved to " + name)
    """

    # Single Test - Walk Through All The Commands
    """
    env.build()
    env.cam.follow_entity(env.robot.robot)
    for c in range(15):
        if env.cam._in_recording is False:
            env.cam.start_recording()
        command = random_command_config()
        env.cfg.command = command
        env.resam_commands()
        obs, _ = env.reset()
        with torch.no_grad():
            for i in tqdm(range(len(env)), desc="Eval..."):
                actions = pi(obs)
                obs, rews, dones, infos = env.step(actions)

        name = cfg.exp_name + "_" + str(cfg.ckpt) + "_" + str(c) + "_eval"

        env.record_visualization(name)
        print("File saved to " + name)
    """


if __name__ == "__main__":
    main(tyro.cli(EvalConfig))
