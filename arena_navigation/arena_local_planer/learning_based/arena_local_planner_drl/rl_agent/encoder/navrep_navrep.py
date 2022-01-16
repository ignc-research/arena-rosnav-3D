from os import path
import numpy as np
import sys

from rl_agent.encoder import BaseEncoder
"""
    NAVREP PRETRAINED MODEL
    Only works for pepper
"""

class NavrepPretrainedEncoder(BaseEncoder):
    def __init__(self, agent_name: str, model_dir: str, hyperparams):
        model_path = path.join(model_dir, agent_name + ".zip")

        assert path.isfile(
            model_path
        ), f"Compressed model cannot be found at {model_path}!"

        self._agent = self._load_model(model_path)
        if hyperparams["normalize"]:
            self._obs_norm_func = self._load_vecnorm()

    def _load_model(self, model_path: str):
        """Import stable baseline here because it requires
            a different python version
        """
        from stable_baselines.ppo2 import PPO2
        import navrep.tools.custom_policy as custom_policy

        sys.modules["custom_policy"] = custom_policy

        return PPO2.load(model_path)

    def _load_vecnorm(self):
        return lambda obs: obs

    def get_observation(self, obs):
        obs_dict = obs[1]
        scan = obs_dict["laser_scan"]
        rho, theta = obs_dict["goal_in_robot_frame"]
        robot_vel = obs_dict["robot_vel"]

        print(scan, np.max(scan))

        # Convert Rho, Theta in robot frame coordinates
        y = np.sin(theta + np.pi) * rho
        x = np.cos(theta + np.pi) * rho

        navrep_observation = np.hstack([scan, [x, y], np.maximum(np.minimum(1, [robot_vel.linear.x, robot_vel.linear.y, 0]), -1)])

        return navrep_observation.reshape(len(navrep_observation), 1)

    def get_action(self, action):
        """
            Encodes the action produced by the nn
            Should always return an array of size 3 with entries
            [x_vel, y_vel, ang_vel]
        """
        assert len(action) == 2, f"Expected an action of size 2 but received {len(action)}: {action}"
        
        x_vel, y_vel = action
        return [x_vel, y_vel, 0]