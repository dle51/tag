import genesis as gs
from rich.pretty import pprint
from tqdm import tqdm
import tyro

from tag.gym.envs.chase.chase_v2 import Chase, ChaseEnvConfig, SingleGymWrapper
from tag.gym.robots.go2 import Go2Config, Go2State
from tag.policy.dummy import DummyPolicy
from tag.utils import spec


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
        "kp": 10.0,
        "kd": 0.25,
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.25,
        "simulate_action_latency": True,
    }

    obs_cfg = {
        "num_obs": 48,
        "obs_scales": {
            "lin_vel": 1.0,
            "ang_vel": 0.5,
            "dof_pos": 1.0,
            "dof_vel": 0.1,
        },
    }

    return env_cfg, obs_cfg


class Config(ChaseEnvConfig):
    cfg_1: Go2Config = Go2Config(
        state=Go2State(
            pos=[0.0, -1.5, 1.42],
            quat=[1.0, 0.0, 0.0, 0.0],
        )
    )
    cfg_2: Go2Config = Go2Config(
        state=Go2State(
            pos=[0.0, 1.5, 1.42],
            quat=[1.0, 0.0, 0.0, 0.0],
        )
    )


def main(cfg: Config):
    gs.init(logging_level="critical")
    cfg.cfgs.append(cfg.cfg_1)
    cfg.cfgs.append(cfg.cfg_2)
    env_cfg, obs_cfg = get_cfgs()

    print("Building Genesis Environment.")
    genesis_env = Chase(cfg, env_cfg=env_cfg, obs_cfg=obs_cfg)
    genesis_env.build()
    print("Success.")

    print("Wrapping in SingleGymWrapper.")
    gym_env = SingleGymWrapper(genesis_env)
    gym_env.env.cam.set_pose(
        pos=[10.0, 0.0, 6.0],
        lookat=[0.0, 0.0, 0.0],
    )
    print("Success.")

    policy = DummyPolicy(gym_env.action_space)

    print("Gymnasium Wrapper Reset Method")
    obs, _ = gym_env.reset()
    print("Success.")

    print("Testing Dummy Policy Stepthrough")
    for i in tqdm(range(200)[:200], desc="Running Dummy Policy"):
        action = policy.act(obs)
        obs, rew, terminated, truncated, infos = gym_env.step(action)
        pprint(spec(action))
        pprint(action)
    print("Success.")

    gym_env.env.cam.stop_recording(save_to_filename="Test.mp4", fps=60)

    print("End Script.")


if __name__ == "__main__":
    main(tyro.cli(Config))
