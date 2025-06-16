from pathlib import Path
import pickle

import genesis as gs
from gymnasium.spaces import Box
import numpy as np
from stable_baselines3.common.vec_env import VecEnv
import torch


class GenesisVecEnv(VecEnv):
    def __init__(self, env):
        self.env = env
        self.num_envs = env.num_envs
        self.device = gs.device

        self.obs_space = Box(low=-np.inf, high=np.inf, shape=(self.env.num_obs,), dtype=np.float32)
        self.act_space = Box(
            low=-self.env.env_cfg["clip_actions"],
            high=self.env.env_cfg["clip_actions"],
            shape=(self.env.num_actions,),
            dtype=np.float32,
        )

        super().__init__(num_envs=self.num_envs, observation_space=self.obs_space, action_space=self.act_space)

    def reset(self):
        obs, _ = self.env.reset()
        return obs.cpu().numpy()

    def step_async(self, actions):
        self._actions = torch.tensor(actions, device=self.device, dtype=torch.float32)

    def step_wait(self):
        obs, rewards, dones, infos = self.env.step(self._actions)

        return (
            obs.cpu().numpy(),
            rewards.cpu().numpy(),
            dones.cpu().numpy().astype(np.bool_),
            infos,
        )

    def close(self):
        del self.env

    def get_attr(self, attr_name, indices=None):
        # Optionally implement for SB3 training introspection
        return getattr(self.env, attr_name)

    def set_attr(self, attr_name, value, indices=None):
        setattr(self.env, attr_name, value)

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        method = getattr(self.env, method_name)
        return method(*method_args, **method_kwargs)

    def render(self, mode="human"):
        self.env.render()

    def env_is_wrapped(self, wrapper_class, indices=None):
        if indices is None:
            indices = list(range(self.num_envs))

        is_wrapped = isinstance(self.env, wrapper_class)
        return [is_wrapped] * len(indices)

    # ADDED: Save method for checkpointing environment state
    def save(self, path):
        """
        Save the environment state to a file.
        This is useful for checkpointing during training.
        """
        save_dict = {
            "num_envs": self.num_envs,
            "device": str(self.device),
            "obs_space": self.obs_space,
            "act_space": self.act_space,
            # Add any other relevant environment state here
        }

        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, "wb") as f:
            pickle.dump(save_dict, f)

    # ADDED: Load method for restoring environment state
    @classmethod
    def load(cls, path, env):
        """
        Load environment state from a file.
        Note: The underlying env object must be provided as it contains
        the actual simulation state which is not easily serializable.
        """
        with open(path, "rb") as f:
            save_dict = pickle.load(f)

        # Create new instance with the provided env
        vec_env = cls(env)
        return vec_env

    # ADDED: Seed method for reproducibility
    def seed(self, seed=None):
        """
        Set random seed for reproducible training.
        """
        if seed is None:
            seed = np.random.randint(0, 2**32 - 1)

        # Set numpy random seed
        np.random.seed(seed)

        # Set torch random seed
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed(seed)
            torch.cuda.manual_seed_all(seed)

        # If the underlying env has a seed method, call it
        if hasattr(self.env, "seed"):
            return self.env.seed(seed)

        return [seed] * self.num_envs

    # FIXED: Step method combining async and wait for convenience
    def step(self, actions):
        """
        Convenience method that combines step_async and step_wait.
        This is the standard interface expected by most RL libraries.
        """
        self.step_async(actions)
        return self.step_wait()

    # ADDED: Get episode statistics for monitoring
    def get_episode_rewards(self):
        """
        Get episode rewards if available from the underlying environment.
        """
        if hasattr(self.env, "episode_rewards"):
            return self.env.episode_rewards
        return None

    def get_episode_lengths(self):
        """
        Get episode lengths if available from the underlying environment.
        """
        if hasattr(self.env, "episode_lengths"):
            return self.env.episode_lengths
        return None

    # ADDED: Additional utility methods for better SB3 compatibility
    def get_spaces(self):
        """
        Return observation and action spaces.
        """
        return self.observation_space, self.action_space

    # ADDED: Context manager support for proper cleanup
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ADDED: String representation for debugging
    def __repr__(self):
        return f"GenesisVecEnv(num_envs={self.num_envs}, obs_shape={self.observation_space.shape}, act_shape={self.action_space.shape})"
