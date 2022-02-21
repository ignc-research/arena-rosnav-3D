from os import path
import numpy as np
import sys

from rl_agent.encoder.factory import EncoderFactory
from rl_agent.encoder import BaseEncoder

"""
    MODELS FOR DIFFERENT ROBOTERS TRAINED IN GULDENRING
"""


class GuldenringEncoder(BaseEncoder):
    def __init__(self, agent_name: str, model_dir: str, hyperparams):
        model_path = path.join(model_dir, agent_name + ".zip")

        assert path.isfile(
            model_path
        ), f"Compressed model cannot be found at {model_path}!"

        self._agent = self._load_model(model_path)
        if hyperparams["normalize"]:
            self._obs_norm_func = self._load_vecnorm()

    def _load_model(self, model_path: str):
        from stable_baselines.ppo2 import PPO2

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

        complete_observation = np.zeros((1, len(scan) + 8 * 2, 1))
        complete_observation[0, : len(scan), 0] = scan

        for i in range(8):
            complete_observation[
                0, len(scan) + i * 2 : len(scan) + i * 2 + 2, 0
            ] = [x, y]

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


class HolonomicRobotEncoder(GuldenringEncoder):
    def get_action(self, action):
        assert (
            len(action) == 3
        ), f"Expected an action of size 3 but received: {action}"

        return action


"""
    Jackal
    N: 720 
    offset: -3/4 * pi
    action: [x_vel, ang_vel]
"""

@EncoderFactory.register("guldenring", "guldenring", "jackal")
class JackalGuldenringEncoder(GuldenringEncoder):
    def get_action(self, action):
        assert len(action) == 2, f"Expected an action of size 2 but received {len(action)}: {action}"
        
        x_vel, ang_vel = action
        return [x_vel, 0, 0.3*ang_vel]


"""
    Turtlebot3
    N: 360
    offset: 0
    action: [x_vel, ang_vel]
"""

@EncoderFactory.register("guldenring", "guldenring", "turtlebot3_burger")
class TurtleBot3BurgerGuldenringEncoder(GuldenringEncoder):
    pass


"""
    Rto
    N: 360
    offset: 0
    action: [x_vel, y_vel, ang_vel]
"""
@EncoderFactory.register("guldenring", "guldenring", "rto")
class RtoGuldenringEncoder(HolonomicRobotEncoder):
    pass

"""
    AGV
    N: 720
    offset: -pi
    action: [x_vel, ang_vel]
"""


@EncoderFactory.register("guldenring", "guldenring", "agv-ota")
class AgvGuldenringEncoder(GuldenringEncoder):
    pass


"""
    Ridgeback
    N: 720
    offset: -3/4 * pi
    action: [x_vel, y_vel, ang_vel]
"""


@EncoderFactory.register("guldenring", "guldenring", "ridgeback")
class RidgebackGuldenringEncoder(HolonomicRobotEncoder):
    pass


# Robots for which models need to be trained
class Cob4Encoder(HolonomicRobotEncoder):
    pass

@EncoderFactory.register("guldenring", "guldenring", "youbot")
class Youbot(HolonomicRobotEncoder):
    pass

class TurtleBot3WaffleGuldenringEncoder(GuldenringEncoder):
    pass

