from os import path
import numpy as np
import sys

from rl_agent.encoder import BaseEncoder
"""
    ROSNAV MODEL TRAINED IN NAVREP ENVIRONMENT
"""

class GuldenringPretrainedEncoder(BaseEncoder):
    def __init__(self, agent_name: str, model_dir: str, hyperparams):
        model_path = path.join(model_dir, agent_name + ".pkl")

        assert path.isfile(
            model_path
        ), f"Compressed model cannot be found at {model_path}!"

        self._agent = self._load_model(model_path)
        if hyperparams["normalize"]:
            self._obs_norm_func = self._load_vecnorm()

    def _load_model(self, model_path: str):
        from stable_baselines.ppo2 import PPO2

        print(model_path)

        return PPO2.load(model_path)

    def _load_vecnorm(self):
        return lambda obs: obs

    def get_observation(self, obs):
        obs_dict = obs[1]
        scan = obs_dict["laser_scan"]
        rho, theta = obs_dict["goal_in_robot_frame"]

        # Convert Rho, Theta in robot frame coordinates
        y = np.sin(theta + np.pi) * rho
        x = np.cos(theta + np.pi) * rho

        complete_observation = np.zeros((1, 90 + 8 * 2, 1))

        downsampled_scan = scan.reshape((-1, 8))
        downsampled_scan = np.min(downsampled_scan, axis=1)

        complete_observation[0, :90, 0] = downsampled_scan

        for i in range(8):
            complete_observation[0, 90 + i * 2:90 + i * 2 + 2, 0] = [x, y]

        guldenring_obs = np.round(np.divide(complete_observation, 0.05))*0.05
        
        return guldenring_obs

    def get_action(self, action):
        """
            Encodes the action produced by the nn
            Should always return an array of size 3 with entries
            [x_vel, y_vel, ang_vel]
        """
        assert len(action) == 2, f"Expected an action of size 2 but received {len(action)}: {action}"
        

        x_vel, ang_vel = action
        return [x_vel, 0, ang_vel]

class JackalGuldenringPretrainedEncoder(GuldenringPretrainedEncoder):
    pass

class TurtleBot3GuldenringPretrainedEncoder(GuldenringPretrainedEncoder):
    pass

class AgvGuldenringPretrainedEncoder(GuldenringPretrainedEncoder):
    pass

class RidgebackGuldenringPretrainedEncoder(GuldenringPretrainedEncoder):
    def get_action(action):
        assert len(action) == 3, f"Expected an action of size 3 but received: {action}"

        return action