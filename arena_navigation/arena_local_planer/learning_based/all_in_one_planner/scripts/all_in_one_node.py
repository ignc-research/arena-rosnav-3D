#!/usr/bin/env python3

import json
import os
import pickle
import sys
import time

import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import Twist
from stable_baselines3 import PPO

from arena_navigation.arena_local_planer.learning_based.all_in_one_planner.scripts.visualizer import AllInOneVisualizer
from local_planner_manager import LocalPlannerManager
from observation_collector import ObservationCollectorAllInOne


class AioNode:
    def __init__(self):
        rospy.loginfo("============= Start all in one planner! =============")
        self.aio_action = 0
        self.cmd_vel = np.zeros((2,))
        self._base_control_interval = int(1e8)

        self._global_planner_counter = 0
        self._aio_switch_counter = 0

        # TODO: Hardcoding this is not great. Get this parameter via the corresponding rosnav hyperprameter?
        rospy.set_param("action_in_obs", True)

        # define paths
        self._robot_model = rospy.get_param("robot_model")
        agent_aio = self._robot_model + "_s+d"

        base_dir = rospkg.RosPack().get_path('all_in_one_planner')
        paths = {'model': os.path.join(base_dir, 'AIO_agents', agent_aio),
                 'drl_agents': os.path.join(base_dir, 'scripts', 'AIO_local_planners', 'rosnav_drl'),
                 'all_in_one_parameters': os.path.join(base_dir, 'AIO_agents', agent_aio, 'all_in_one_parameters.json')}

        if not os.path.exists(paths['model']):
            rospy.logerr("No AIO planner defined for robot model " + self._robot_model + "! Exit")
            sys.exit(1)

        # TODO: Load goal radius? Requirements?
        self._goal_radius = 0.1

        rospy.loginfo("Loading all in one policy from file...")

        self.load_aio_policy(paths['model'])

        self.local_planner_manager = LocalPlannerManager(paths)

        required_obs = self.local_planner_manager.get_required_observations()
        self.observation_collector = ObservationCollectorAllInOne(required_obs, paths['all_in_one_parameters'])
        self._extract_step_parameters(paths['all_in_one_parameters'])

        self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.visualizer = AllInOneVisualizer(self.local_planner_manager.get_model_names())

        rospy.loginfo("All in one planner: All modules loaded!")

        rospy.sleep(2)

        self.aio_timer = rospy.Timer(rospy.Duration(0, self._base_control_interval), self.aio_control_sequence)

        rospy.loginfo("============= All in one planner ready! =============")

    def load_aio_policy(self, model_path):
        vecnorm_path = os.path.join(model_path, "vec_normalize.pkl")
        model_parameters_path = os.path.join(model_path, "best_model.zip")
        assert os.path.isfile(model_parameters_path), "No model file found in %s" % model_parameters_path
        assert os.path.isfile(vecnorm_path), "No vecnorm file found in %s" % vecnorm_path

        # load vec norm
        with open(os.path.join(model_path, "vec_normalize.pkl"), "rb") as file_handler:
            vec_normalize = pickle.load(file_handler)

        self._obs_norm_func = vec_normalize.normalize_obs

        # Magic for loading under python3.8+ for models trained under 3.6/3.7: use custom object
        # For reference: https://github.com/DLR-RM/stable-baselines3/pull/336
        custom_objects = {
            "learning_rate": 0.0,
            "lr_schedule": lambda _: 0.0,
            "clip_range": lambda _: 0.0,
        }
        # load agent
        self._aio_policy = PPO.load(model_parameters_path, custom_objects=custom_objects).policy

    def stop_moving(self):
        print("Reached goal - Stop AIO planner.")
        twist = Twist()
        self.pub_twist.publish(twist)

    def publish_cmd_vel(self):
        action_msg = Twist()
        if len(self.cmd_vel) == 2:
            action_msg.linear.x = self.cmd_vel[0]
            action_msg.angular.z = self.cmd_vel[1]
        else:
            action_msg.linear.x = self.cmd_vel[0]
            action_msg.linear.y = self.cmd_vel[1]
            action_msg.angular.z = self.cmd_vel[2]
        self.pub_twist.publish(action_msg)

    def aio_control_sequence(self, _):
        start_time = rospy.get_rostime()

        if self._global_planner_counter % self._update_global_plan_frequency == 0:
            make_new_global_plan = True
        else:
            make_new_global_plan = False

        merged_obs, obs_dict = self.observation_collector.get_observations(make_new_global_plan)

        goal_reached = rospy.get_param("/bool_goal_reached", default=False)
        if not goal_reached:
            if self._aio_switch_counter % self._all_in_one_planner_frequency == 0:
                self.aio_action = self._get_aio_action(merged_obs)

            self.cmd_vel = self.local_planner_manager.execute_local_planner(self.aio_action, obs_dict)
            self.publish_cmd_vel()
            self.visualizer.visualize_step(self.aio_action, obs_dict['robot_pose'])
        else:
            self.stop_moving()
        self._global_planner_counter += 1
        self._aio_switch_counter += 1

        end_time = rospy.get_rostime()
        time_interval_ms = (end_time.nsecs - start_time.nsecs) * 1e-6

        if time_interval_ms > 100:
            rospy.logwarn(
                "AIO Planner: Time interval of 100ms exceeded! " + str(time_interval_ms) + "ms needed!")

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down Node.")
        self.local_planner_manager.close_planners()
        self.stop_moving()

    def _extract_step_parameters(self, config_path: str):
        with open(config_path, 'r') as config_json:
            config_data = json.load(config_json)

        assert config_data is not None, "Error: All in one parameter file cannot be found!"

        # extract update rate of global planner
        if 'update_global_plan_frequency' in config_data:
            self._update_global_plan_frequency = config_data['update_global_plan_frequency']
        else:
            rospy.logwarn(
                "Parameter \"update_global_plan_frequency\" not found in config file. Use default value of 5!")
            self._update_global_plan_frequency = 4
        # extract frequency of all in one planner
        if 'all_in_one_planner_frequency' in config_data:
            self._all_in_one_planner_frequency = config_data['all_in_one_planner_frequency']
        else:
            rospy.logwarn(
                'Parameter \"all_in_one_planner_frequency\" not found in config file. Us edefault value of 5!')
            self._all_in_one_planner_frequency = 4

    def goal_reached(self, obs_dict: dict):
        if obs_dict['global_goal_robot_frame'][0] < self._goal_radius:
            return True
        else:
            return False

    def _get_aio_action(self, merged_obs):
        obs_normed = self._obs_norm_func(merged_obs)
        return self._aio_policy.predict(obs_normed, deterministic=True)[0]


def run():
    rospy.init_node('aio_planner', anonymous=False)

    aio_routine = AioNode()
    rospy.on_shutdown(aio_routine.on_shutdown)

    rospy.spin()


if __name__ == '__main__':
    run()
