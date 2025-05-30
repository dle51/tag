from pathlib import Path

import genesis as gs
from rich.pretty import pprint
import torch
from tqdm import tqdm
import tyro

from tag.gym.envs.chase.chase import Chase, ChaseEnvConfig
from tag.policy.dummy import DummyPolicy
from tag.utils import batch_space


# NOTE(dle): Implement a direct way of doing this
def look_at_each_other(robots):
    robots[1].robot.robot.set_quat([0.7071, 0.0, 0.0, 0.7071])
    robots[2].robot.robot.set_quat([0.7071, 0.0, 0.0, -0.7071])
    return


# NOTE(dle): Implement in class
def check_for_collisions(robots):
    contacts = robots[1].robot.robot.get_contacts(with_entity=robots[2].robot.robot)["geom_a"]
    return torch.any(contacts != 0)


def main(cfg: ChaseEnvConfig):
    gs.init(logging_level="info", backend=gs.gpu)

    pprint(cfg)

    env = Chase(cfg)
    env.build()

    policy = DummyPolicy(batch_space(env.action_space, env.B))

    obs, _ = env.reset()
    look_at_each_other(env.robot.robots)
    counter = 0
    for i in tqdm(range(len(env))[:200], desc="Running Dummy Policy"):
        action = policy.act(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        if check_for_collisions(env.robot.robots):
            counter += 1
            obs, _ = env.reset()
            look_at_each_other(env.robot.robots)

    env.record_visualization(Path(__file__).with_suffix(".mp4"))
    print(counter)


if __name__ == "__main__":
    main(tyro.cli(ChaseEnvConfig))
