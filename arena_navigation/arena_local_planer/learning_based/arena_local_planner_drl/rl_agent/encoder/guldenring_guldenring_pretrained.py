from os import path
from turtle import down
import numpy as np
import sys
from scipy import interpolate

from rl_agent.encoder.factory import EncoderFactory
from rl_agent.encoder import BaseEncoder

"""
    GULDENRING PRETRAINED MODELS
    Only works for agv-ota
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

    def get_observation(self, obs, *args, **kwargs):
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
            complete_observation[0, 90 + i * 2 : 90 + i * 2 + 2, 0] = [x, y]

        return np.round(np.divide(complete_observation, 0.05)) * 0.05

    def get_action(self, action):
        """
        Encodes the action produced by the nn
        Should always return an array of size 3 with entries
        [x_vel, y_vel, ang_vel]
        """
        assert (
            len(action) == 2
        ), f"Expected an action of size 2 but received {len(action)}: {action}"

        x_vel, ang_vel = action
        return [x_vel, 0, ang_vel]

@EncoderFactory.register("guldenring", "guldenring_pretrained", "turtlebot3_burger")
class TurtleBot3Encoder(GuldenringPretrainedEncoder):
    def get_observation(self, obs):
        obs_dict = obs[1]
        scan = obs_dict["laser_scan"]
        rho, theta = obs_dict["goal_in_robot_frame"]

        # Convert Rho, Theta in robot frame coordinates
        y = np.sin(theta + np.pi) * rho
        x = np.cos(theta + np.pi) * rho

        complete_observation = np.zeros((1, 90 + 8 * 2, 1))

        downsampled_scan = scan.reshape((-1, 2))
        downsampled_scan = np.min(downsampled_scan, axis=1)

        complete_observation[0, :45, 0] = downsampled_scan[135:]
        complete_observation[0, 45:90, 0] = downsampled_scan[:45]

        for i in range(8):
            complete_observation[0, 90 + i * 2 : 90 + i * 2 + 2, 0] = [x, y]

        guldenring_obs = np.round(np.divide(complete_observation, 0.05)) * 0.05

        return guldenring_obs

@EncoderFactory.register("guldenring", "guldenring_pretrained", "jackal")
class JackalEncoder(GuldenringPretrainedEncoder):
    def get_observation(self, obs):
        obs_dict = obs[1]
        scan = obs_dict["laser_scan"]
        rho, theta = obs_dict["goal_in_robot_frame"]

        # Convert Rho, Theta in robot frame coordinates
        y = np.sin(theta + np.pi) * rho
        x = np.cos(theta + np.pi) * rho

        rotated_scan = np.zeros((480))
        rotated_scan[:240] = scan[120:360]
        rotated_scan[240:] = scan[360:600]

        downsampled_scan = rotated_scan.reshape((-1, 5))
        downsampled_scan = np.min(downsampled_scan, axis=1)

        f = interpolate.interp1d(np.arange(0, 96), downsampled_scan)
        upsampled = f(np.linspace(0, 96 - 1, 90))

        complete_observation = np.zeros((1, 90 + 8 * 2, 1))

        downsampled_scan = scan.reshape((-1, 2))
        downsampled_scan = np.min(downsampled_scan, axis=1)

        complete_observation[0, :90, 0] = upsampled

        for i in range(8):
            complete_observation[0, 90 + i * 2 : 90 + i * 2 + 2, 0] = [x, y]

        guldenring_obs = np.round(np.divide(complete_observation, 0.05)) * 0.05

        return guldenring_obs
