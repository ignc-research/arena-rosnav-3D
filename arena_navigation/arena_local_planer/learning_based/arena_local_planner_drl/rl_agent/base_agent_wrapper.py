from abc import ABC, abstractmethod
from typing import Tuple

import json
import numpy as np
import os
import rospy
import rospkg
import yaml

from gym import spaces

from geometry_msgs.msg import Twist

from arena_navigation.arena_local_planer.learning_based.arena_local_planner_drl.rl_agent.utils.observation_collector import (
    ObservationCollector,
)
from rl_agent.utils.reward import RewardCalculator
from rospy.client import get_param

robot_model = rospy.get_param("model")
ROOT_ROBOT_PATH = os.path.join(
    rospkg.RosPack().get_path("simulator_setup"), "robot"
)
DEFAULT_ACTION_SPACE = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    f"default_settings_{robot_model}.yaml",
)
DEFAULT_HYPERPARAMETER = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    "hyperparameters",
    "default.json",
)
DEFAULT_NUM_LASER_BEAMS, DEFAULT_LASER_RANGE = 360, 3.5
GOAL_RADIUS = 0.33


class BaseDRLAgent(ABC):
    def __init__(
        self,
        ns: str = None,
        robot_name: str = None,
        hyperparameter_path: str = DEFAULT_HYPERPARAMETER,
        action_space_path: str = DEFAULT_ACTION_SPACE,
        *args,
        **kwargs,
    ) -> None:
        """ [summary]

            Args:
                ns (str, optional):
                    Robot specific ROS namespace extension. 
                    Defaults to None.
                robot_name (str, optional):
                    Agent name (directory has to be of the same name). 
                    Defaults to None.
                hyperparameter_path (str, optional):
                    Path to json file containing defined hyperparameters.
                    Defaults to DEFAULT_HYPERPARAMETER.
                action_space_path (str, optional):
                    Path to yaml file containing action space settings.
                    Defaults to DEFAULT_ACTION_SPACE.
        """

        # Setup node namespace
        self._ns = BaseDRLAgent._create_namespace(ns, robot_name)
        self._sim_ns = robot_name

        # Load robot and model specific parameters 
        self._hyperparams = BaseDRLAgent._load_hyperparameters(path=hyperparameter_path)
        self._num_laser_beams, self._laser_range, self._robot_radius = \
            BaseDRLAgent._get_robot_settings(self._sim_ns)
        self._discrete_actions, self._continuous_actions, self._is_holonomic = \
            BaseDRLAgent._read_action_space(action_space_path)

        self._action_space = self._get_action_space()
        
        self._reward_calculator = self._create_reward_calculator()

        self._observation_collector = ObservationCollector(
            self._ns, self._num_laser_beams, self._laser_range, True
        )

        # for time controlling in train mode
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate")

        self._is_train_mode = rospy.get_param("/train_mode")
        # TODO
        if self._is_train_mode:
            # w/o action publisher node
            self._action_pub = rospy.Publisher(
                f"{self._ns}cmd_vel", Twist, queue_size=1
            )
        else:
            # w/ action publisher node
            # (controls action rate being published on '../cmd_vel')
            self._action_pub = rospy.Publisher(
                # f"{self._ns}cmd_vel_pub", Twist, queue_size=1
                f"{self._ns}cmd_vel",
                Twist,
                queue_size=1,
            )

    def _create_reward_calculator(self) -> None:
        """Sets up the reward calculator."""
        assert self._hyperparams and "reward_fnc" in self._hyperparams
        return RewardCalculator(
            robot_radius=self._robot_radius,
            safe_dist=1.6 * self._robot_radius,
            goal_radius=GOAL_RADIUS,
            rule=self._hyperparams["reward_fnc"],
            extended_eval=False,
        )

    def get_observations(self) -> Tuple[np.ndarray, dict]:
        """
            Retrieves the latest synchronized observation.

            Returns:
                Tuple, where first entry depicts the observation data concatenated \
                into one array. Second entry represents the observation dictionary.
        """
        merged_obs, obs_dict = self._observation_collector.get_observations()
        if self._hyperparams["normalize"]:
            self.normalize_observations(merged_obs)
        return merged_obs, obs_dict

    def normalize_observations(self, merged_obs: np.ndarray) -> np.ndarray:
        """
            Normalizes the observations with the loaded VecNormalize object.

            Note:
                VecNormalize object from Stable-Baselines3 is agent specific\
                and integral part in order to map right actions.\

            Args:
                merged_obs (np.ndarray):
                    observation data concatenated into one array.

            Returns:
                np.ndarray: Normalized observations array.
        """
        assert self._hyperparams["normalize"] and hasattr(
            self, "_obs_norm_func"
        )
        return self._obs_norm_func(merged_obs)
    
    def publish_action(self, action: np.ndarray) -> None:
        """
            TODO
            Publishes an action on 'self._action_pub' (ROS topic).

            Args:
                action (np.ndarray):
                    For none holonomic robots action is [xVel, angularVel]
                    For holonomic robots action is [xVel, yVel, angularVel]
                    xVel and yVel in m/s, angularVel in rad/s
        """
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]



        # action_msg = Twist()
        # action_msg.linear.x = action[0]

        # if self._is_holonomic:
        #     assert (
        #         len(action) == 3
        #     ), "Holonomic robots require action arrays to have 3 entries."
        #     action_msg.linear.y = action[1]
        #     action_msg.angular.z = action[2]
        # else:
        #     assert (
        #         len(action) == 2
        #     ), "Non-holonomic robots require action arrays to have 2 entries."
        #     action_msg.angular.z = action[1]

        self._action_pub.publish(action_msg)

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        """
            Infers an action based on the given observation.

            Args:
                obs (np.ndarray): Merged observation array.

            Returns:
                np.ndarray:
                    Action in [linear velocity, angular velocity]
        """
        assert self._agent, "Agent model not initialized!"
        action = self._agent.predict(obs, deterministic=True)[0]
        if self._hyperparams["discrete_action_space"]:
            action = self._get_disc_action(action)
        else:
            # clip action
            action = np.maximum(
                np.minimum(self._action_space.high, action),
                self._action_space.low,
            )
        return action

    def get_reward(self, action: np.ndarray, obs_dict: dict) -> float:
        """
            Calculates the reward based on the parsed observation

            Args:
                action (np.ndarray):
                    Velocity commands of the agent \
                    in [linear velocity, angular velocity].
                obs_dict (dict):
                    Observation dictionary where each key makes up a different \
                    kind of information about the environment.
            Returns:
                float: Reward amount
        """
        return self._reward_calculator.get_reward(action=action, **obs_dict)

    def _get_disc_action(self, action: int) -> np.ndarray:
        """Returns defined velocity commands for parsed action index.\
            (Discrete action space)

        Args:
            action (int): Index of the desired action.

        Returns:
            np.ndarray: Velocity commands corresponding to the index.
        """
        return np.array(
            [
                self._discrete_actions[action]["linear"],
                self._discrete_actions[action]["angular"],
            ]
        )

    def _get_action_space(self) -> None:
        """Sets up the action space. (spaces.Box)"""
        assert self._discrete_actions or self._continuous_actions
        assert (
            self._hyperparams and "discrete_action_space" in self._hyperparams
        )

        if self._hyperparams["discrete_action_space"]:
            # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
            assert (
                not self._is_holonomic
            ), "Discrete action space currently not supported for holonomic robots"

            return spaces.Discrete(len(self._discrete_actions))
        else:
            linear_range = self._continuous_actions[
                "linear_range"
            ]
            angular_range = self._continuous_actions[
                "angular_range"
            ]

            if not self._is_holonomic:
                return spaces.Box(
                    low=np.array([linear_range[0], angular_range[0]]),
                    high=np.array([linear_range[1], angular_range[1]]),
                    dtype=np.float,
                )
            else:
                linear_range_x, linear_range_y = (
                    linear_range["x"],
                    linear_range["y"],
                )
                return spaces.Box(
                    low=np.array(
                        [
                            linear_range_x[0],
                            linear_range_y[0],
                            angular_range[0],
                        ]
                    ),
                    high=np.array(
                        [
                            linear_range_x[1],
                            linear_range_y[1],
                            angular_range[1],
                        ]
                    ),
                    dtype=np.float,
                )

    @property
    def action_space(self) -> spaces.Box:
        return self._action_space

    @property
    def observation_space(self) -> spaces.Box:
        return self._observation_collector.observation_space

    @abstractmethod
    def _setup_agent(self) -> None:
        """
            Sets up the new agent / loads a pretrained one.

            Raises:
                NotImplementedError: Abstract method.
        """
        raise NotImplementedError

    
    @staticmethod
    def _create_namespace(ns: str, robot_name: str):
        ns = "" if ns is None or ns == "" else ns + "/"

        if robot_name == None:
            return ns
        return ns + robot_name + "/"

    @staticmethod
    def _load_hyperparameters(path: str) -> None:
        """
            Path should point to a file containing the
            specific hyperparameters used for training
        """
        assert os.path.isfile(
            path
        ), f"Hyperparameters file cannot be found at {path}!"

        with open(path, "r") as file:
            hyperparams = json.load(file)

        return hyperparams

    @staticmethod
    def _get_robot_settings(ns: str):
        """
            Setup robot specific parameters by reading ros params
        """
        _num_laser_beams = None
        _laser_range = None

        _robot_radius = rospy.get_param("radius") * 1.05
        _num_laser_beams = get_param("laser_beams")
        _laser_range = rospy.get_param("laser_range")

        if _num_laser_beams is None:
            _num_laser_beams = DEFAULT_NUM_LASER_BEAMS
            print(
                f"{ns}:"
                "Wasn't able to read the number of laser beams."
                "Set to default: {DEFAULT_NUM_LASER_BEAMS}"
            )
        if _laser_range is None:
            _laser_range = DEFAULT_LASER_RANGE
            print(
                f"{ns}:"
                "Wasn't able to read the laser range."
                "Set to default: {DEFAULT_LASER_RANGE}"
            )

        return _num_laser_beams, _laser_range, _robot_radius

    @staticmethod
    def _read_action_space(
         action_space_yaml_path: str
    ) -> None:
        """
            Retrieves the robot action space from respective yaml file.

            Args:
                action_space_yaml_path (str):
                    Yaml file containing the action space configuration.
        """
        assert os.path.isfile(
            action_space_yaml_path
        ), f"Action space file cannot by found at {action_space_yaml_path}"

        with open(action_space_yaml_path, "r") as fd:
            setting_data = yaml.safe_load(fd)

            return (
                setting_data["robot"]["discrete_actions"], 
                {
                    "linear_range": setting_data["robot"]["continuous_actions"][
                        "linear_range"
                    ],
                    "angular_range": setting_data["robot"]["continuous_actions"][
                        "angular_range"
                    ],
                },
                setting_data["robot"]["holonomic"]
            )
