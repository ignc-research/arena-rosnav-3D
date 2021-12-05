#! /usr/bin/env python
from operator import is_
from random import randint
import gym
from gym import spaces
from gym.spaces import space
from typing import Union
from stable_baselines3.common.env_checker import check_env
import yaml
from rl_agent.utils.observation_collector import ObservationCollector
from rl_agent.utils.reward import RewardCalculator
from rl_agent.utils.debug import timeit
from rospy.exceptions import ROSException
from task_generator.task_generator.tasks import ABSTask
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String
# from flatland_msgs.srv import StepWorld, StepWorldRequest #resolve later
from std_msgs.msg import Bool
import time
import math
import xml.etree.ElementTree as ET
from std_srvs.srv import Empty

from rl_agent.utils.debug import timeit
from task_generator.task_generator.tasks import *


class GazeboEnv(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(
        self,
        ns: str,
        reward_fnc: str,
        is_action_space_discrete,
        safe_dist: float = None,
        goal_radius: float = 0.1,
        max_steps_per_episode=100,
        train_mode: bool = True,
        debug: bool = False,
        task_mode: str = "staged",
        PATHS: dict = dict(),
        extended_eval: bool = False,
        *args,
        **kwargs,
    ):
        """Default env
        Gazebo yaml node check the entries in the yaml file, therefore other robot related parameters cound only be saved in an other file.
        TODO : write an uniform yaml paser node to handel with multiple yaml files.


        Args:
            task (ABSTask): [description]
            reward_fnc (str): [description]
            train_mode (bool): bool to differ between train and eval env during training
            is_action_space_discrete (bool): [description]
            safe_dist (float, optional): [description]. Defaults to None.
            goal_radius (float, optional): [description]. Defaults to 0.1.
            extended_eval (bool): more episode info provided, no reset when crashing
        """
        super(GazeboEnv, self).__init__()

        self.ns = ns
        try:
            # given every environment enough time to initialize, if we dont put sleep,
            # the training script may crash.
            ns_int = int(ns.split("_")[1])
            time.sleep(ns_int * 2)
        except Exception:
            rospy.logwarn(
                f"Can't not determinate the number of the environment, training script may crash!"
            )

        # process specific namespace in ros system
        self.ns_prefix = "" if (ns == "" or ns is None) else "/" + ns + "/"

        if not debug:
            if train_mode:
                rospy.init_node(f"train_env_{self.ns}", disable_signals=False)
            else:
                rospy.init_node(f"eval_env_{self.ns}", disable_signals=False)

        self._extended_eval = extended_eval
        self._is_train_mode = rospy.get_param("/train_mode")
        self._is_action_space_discrete = is_action_space_discrete

        self.setup_by_configuration(PATHS["robot_setting"], PATHS["robot_as"])

        # Added to step through the gazebo simulation
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

        # observation collector
        self.observation_collector = ObservationCollector(
            self.ns,
            self._laser_num_beams,
            self._laser_max_range,
            external_time_sync=True,        
        )
        self.observation_space = (
            self.observation_collector.get_observation_space()
        )

        # reward calculator
        if safe_dist is None:
            safe_dist = 1.6 * self._robot_radius

        self.reward_calculator = RewardCalculator(
            robot_radius=self._robot_radius,
            safe_dist=1.6 * self._robot_radius,
            goal_radius=goal_radius,
            rule=reward_fnc,
            extended_eval=self._extended_eval,
        )

        # action agent publisher
        if self._is_train_mode:
            self.agent_action_pub = rospy.Publisher(
                f"{self.ns_prefix}cmd_vel", Twist, queue_size=1
            )
        else:
            self.agent_action_pub = rospy.Publisher(
                f"{self.ns_prefix}cmd_vel_pub", Twist, queue_size=1
            )

        # service clients
        if self._is_train_mode:
            self._service_name_step = f"{self.ns_prefix}step_world"
            # self._sim_step_client = rospy.ServiceProxy(   #resolve later TODO
            #     self._service_name_step, StepWorld        #resolve later
            # )                                             #resolve later

        # instantiate task manager
        self.task = get_predefined_task(
            ns, mode=task_mode, start_stage=kwargs["curr_stage"], PATHS=PATHS
        )

        self._steps_curr_episode = 0
        self._max_steps_per_episode = max_steps_per_episode

        # for extended eval
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate")
        self._last_robot_pose = None
        self._distance_travelled = 0
        self._safe_dist_counter = 0
        self._collisions = 0
        self._in_crash = False

    def setup_by_configuration(
        self, robot_xml_path: str, settings_yaml_path: str
    ):
        """get the configuration from the xml file, including robot radius, discrete action space and continuous action space.

        Args:
            robot_xml_path (str): [description]
            settings_yaml_path (str): [description]
        """
        # get robot radius
        self._robot_radius = rospy.get_param('radius')

        # get laser related information
        # tree = ET.parse(robot_xml_path) #change name
        # root = tree.getroot()
        # if 'ray' in root.find(".//ray").tag:
        #     self._laser_num_beams = int(root.find('.//samples').text) # num of laser beams
        #     self._laser_max_range = root.find('.//max').text
        self._laser_num_beams = rospy.get_param('laser_beams')
        self._laser_max_range = rospy.get_param('laser_range')

        with open(settings_yaml_path, "r") as fd: 
            setting_data = yaml.safe_load(fd)
            if self._is_action_space_discrete:
                # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
                self._discrete_acitons = setting_data["robot"][
                    "discrete_actions"
                ]
                self.action_space = spaces.Discrete(len(self._discrete_acitons))
            else:
                linear_range = setting_data["robot"]["continuous_actions"][
                    "linear_range"
                ]
                angular_range = setting_data["robot"]["continuous_actions"][
                    "angular_range"
                ]
                self.action_space = spaces.Box(
                    low=np.array([linear_range[0], angular_range[0]]),
                    high=np.array([linear_range[1], angular_range[1]]),
                    dtype=np.float,
                )

    def _pub_action(self, action):
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        self.agent_action_pub.publish(action_msg)

    def _translate_disc_action(self, action):
        new_action = np.array([])
        new_action = np.append(
            new_action, self._discrete_acitons[action]["linear"]
        )
        new_action = np.append(
            new_action, self._discrete_acitons[action]["angular"]
        )

        return new_action

    def step(self, action):
        """
        Function executed each time step.
        Here we get the action execute it in a time step and retrieve the
        observations generated by that action.
        done_reasons:   0   -   exceeded max steps
                        1   -   collision with obstacle
                        2   -   goal reached
        """
        if self._is_action_space_discrete:
            action = self._translate_disc_action(action)
        self._pub_action(action)
        # print(f"Linear: {action[0]}, Angular: {action[1]}")
        self._steps_curr_episode += 1

        # unpause gazebo to collect data
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # wait for new observations
        merged_obs, obs_dict = self.observation_collector.get_observations()

        # pause simualtion to compute reward
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict["laser_scan"],
            obs_dict["goal_in_robot_frame"],
            action=action,
            global_plan=obs_dict["global_plan"],
            robot_pose=obs_dict["robot_pose"],
        )
        # print(f"cum_reward: {reward}")
        done = reward_info["is_done"]

        # extended eval info
        if self._extended_eval:
            self._update_eval_statistics(obs_dict, reward_info)

        # info
        info = {}

        if done:
            info["done_reason"] = reward_info["done_reason"]
            info["is_success"] = reward_info["is_success"]

        if self._steps_curr_episode > self._max_steps_per_episode:
            done = True
            info["done_reason"] = 0
            info["is_success"] = 0

        # for logging
        if self._extended_eval and done:
            info["collisions"] = self._collisions
            info["distance_travelled"] = round(self._distance_travelled, 2)
            info["time_safe_dist"] = (
                self._safe_dist_counter * self._action_frequency
            )
            info["time"] = self._steps_curr_episode * self._action_frequency
        return merged_obs, reward, done, info

    def reset(self):
        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self.agent_action_pub.publish(Twist())
        # if self._is_train_mode:       #resolve later TODO
            # self._sim_step_client()   #resolve later - This starts the simulation when reset. Where is it stopped?
        self.task.reset()
        self.reward_calculator.reset()
        self._steps_curr_episode = 0

        # extended eval info
        if self._extended_eval:
            self._last_robot_pose = None
            self._distance_travelled = 0
            self._safe_dist_counter = 0
            self._collisions = 0



        obs, _ = self.observation_collector.get_observations()

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        return obs  # reward, done, info can't be included

    def close(self):
        pass

    def _update_eval_statistics(self, obs_dict: dict, reward_info: dict):
        """
        Updates the metrics for extended eval mode

        param obs_dict (dict): observation dictionary from ObservationCollector.get_observations(),
            necessary entries: 'robot_pose'
        param reward_info (dict): dictionary containing information returned from RewardCalculator.get_reward(),
            necessary entries: 'crash', 'safe_dist'
        """
        # distance travelled
        if self._last_robot_pose is not None:
            self._distance_travelled += GazeboEnv.get_distance(
                self._last_robot_pose, obs_dict["robot_pose"]
            )

        # collision detector
        if "crash" in reward_info:
            if reward_info["crash"] and not self._in_crash:
                self._collisions += 1
                # when crash occures, robot strikes obst for a few consecutive timesteps
                # we want to count it as only one collision
                self._in_crash = True
        else:
            self._in_crash = False

        # safe dist detector
        if "safe_dist" in reward_info and reward_info["safe_dist"]:
            self._safe_dist_counter += 1

        self._last_robot_pose = obs_dict["robot_pose"]

    @staticmethod
    def get_distance(pose_1: Pose2D, pose_2: Pose2D):
        return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)


if __name__ == "__main__":

    rospy.init_node("gazebo_gym_env", anonymous=True, disable_signals=False)
    print("start")

    gazebo_env = GazeboEnv()
    check_env(gazebo_env, warn=True)

    # init env
    obs = gazebo_env.reset()

    # run model
    n_steps = 200
    for _ in range(n_steps):
        # action, _states = model.predict(obs)
        action = gazebo_env.action_space.sample()

        obs, rewards, done, info = gazebo_env.step(action)

        time.sleep(0.1)
