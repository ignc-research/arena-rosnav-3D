from os import path
import numpy as np
import pickle
import rospy

from rl_agent.encoder import BaseEncoder
from rl_agent.encoder.factory import EncoderFactory

"""
    ROSNAV MODEL TRAINED IN ROSNAV ENVIRONMENT
"""

"""
    The input of every rosnav agent is structured as following:
    [...observation, rho, theta] 
    Observation:
        Array of laser scan values of size N, N differs for every
        robot and matches the properties of the hardware. The
        array consists of floating point numbers. The offset
        descripes the starting point of the observation and is
        in rad
    Rho:
        floating point number representing the distance between
        the robot and the current target
    Theta:
        floating point number representing the angle between
        the robot and the current target in the robot frame

    The output of every robot differs and matches the size matches
    the degrees of freedom
"""


class RosnavEncoder(BaseEncoder):
    def __init__(
        self, agent_name: str, model_dir: str, hyperparams, *args, **kwargs
    ):
        model_path = path.join(model_dir, agent_name, "best_model.zip")
        vecnorm_path = path.join(model_dir, agent_name, "vec_normalize.pkl")

        self._action_in_obs = hyperparams.get(
            "actions_in_observationspace", False
        )
        rospy.set_param("action_in_obs", self._action_in_obs)
        # import to update the input size of the models
        import rl_agent.model.feature_extractors
        import rl_agent.model.custom_policy

        assert path.isfile(
            model_path
        ), f"Compressed model cannot be found at {model_path}!"

        self._agent = self._load_model(model_path)
        if hyperparams["normalize"]:
            assert path.isfile(
                vecnorm_path
            ), f"VecNormalize file cannot be found at {vecnorm_path}!"

            self._obs_norm_func = self._load_vecnorm(vecnorm_path)

    def _load_model(self, model_path: str):
        # KEEP THIS IMPORT HERE
        # Moving this on top will break navrep and guldenring
        from stable_baselines3 import PPO

        return PPO.load(model_path).policy

    def _load_vecnorm(self, vecnorm_path: str):
        with open(vecnorm_path, "rb") as file_handler:
            return pickle.load(file_handler).normalize_obs

    def get_observation(self, obs: np.ndarray, *args, **kwargs):
        merged_obs, obs_dict = obs
        if not self._action_in_obs:
            return merged_obs
        else:
            return np.hstack([merged_obs, obs_dict["last_action"]])

    def get_action(self, action: np.ndarray) -> list:
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


class RosnavHolonomicEncoder(RosnavEncoder):
    def get_action(self, action):
        assert (
            len(action) == 3
        ), f"Expected an action of size 3 but received: {action}"

        return action


@EncoderFactory.register("rosnav", "rosnav", "jackal")
class JackalEncoder(RosnavEncoder):
    """
    Jackal
    N: 720
    offset: -3/4 * pi
    action: [x_vel, ang_vel]
    """

    pass


@EncoderFactory.register("rosnav", "rosnav", "turtlebot3_burger")
class TurtleBot3BurgerEncoder(RosnavEncoder):
    """
    Turtlebot3
    N: 360
    offset: 0
    action: [x_vel, ang_vel]
    """

    pass

@EncoderFactory.register("rosnav", "rosnav", "turtlebot3_burger_pi")
class TurtleBot3WafflePiEncoder(RosnavEncoder):
    """
    Turtlebot3
    N: 360
    offset: 0
    action: [x_vel, ang_vel]
    """

    pass


@EncoderFactory.register("rosnav", "rosnav", "agv-ota")
class AgvEncoder(RosnavEncoder):
    """
    AGV
    N: 720
    offset: -pi
    action: [x_vel, ang_vel]
    """

    pass


@EncoderFactory.register("rosnav", "rosnav", "tiago")
class TiagoEncoder(RosnavEncoder):
    """
    Tiago
    N: 720
    offset: -pi
    action: [x_vel, ang_vel]
    """

    pass

"""
    HOLONOMIC ROBOTS
"""

@EncoderFactory.register("rosnav", "rosnav", "rto")
class RtoEncoder(RosnavHolonomicEncoder):
    """
    RTO
    N: 720
    offset: -pi
    action: [x_vel, y_vel, ang_vel]
    """

    pass


@EncoderFactory.register("rosnav", "rosnav", "cob4")
class Cob4Encoder(RosnavHolonomicEncoder):
    """
    Care-O-Bot 4
    N: 720
    offset: -pi
    action: [x_vel, y_vel, ang_vel]
    """

    pass


@EncoderFactory.register("rosnav", "rosnav", "ridgeback")
class RidgebackEncoder(RosnavHolonomicEncoder):
    """
    Ridgeback
    N: 720
    offset: -3/4 * pi
    action: [x_vel, y_vel, ang_vel]
    """

    pass

@EncoderFactory.register("rosnav", "rosnav", "youbot")
class YoubotEncoder(RosnavHolonomicEncoder):
    """
    Youbot
    N: 512
    offset: -pi / 2
    action: [x_vel, y_vel, ang_vel]
    """
    pass