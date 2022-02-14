import json
import os
import pickle
from typing import Dict

import numpy as np
import rospkg
from AIO_local_planners.model_base_class import ModelBase
from stable_baselines3 import PPO

# NOTE: Is is assumed that the input laser for the drl planner is:
# angle: {min: -1.5707963267948966, max: 4.694936014, increment:  0.017453292}
import rospy


class DrlAgent(ModelBase):

    def __init__(self, save_path: str, name: str):
        observation_information = {'laser_scan': True,
                                   'goal_in_robot_frame': True}
        super(DrlAgent, self).__init__(observation_information, "rosnav-drl")

        robot_model = rospy.get_param("robot_model")
        base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
        agent_dir = os.path.join(base_dir, 'agents', 'rosnav', robot_model)

        model_save_path = os.path.join(agent_dir, "best_model")
        vec_norm_save_path = os.path.join(agent_dir, "vec_normalize")

        if os.path.exists(vec_norm_save_path):
            with open(vec_norm_save_path, "rb") as file_handler:
                vec_normalize = pickle.load(file_handler)
            self._obs_norm_func = vec_normalize.normalize_obs
            self._obs_normalize = True
        else:
            self._obs_normalize = False

        self._model = PPO.load(model_save_path).policy

    def get_next_action(self, observation_dict, observation_array=None) -> np.ndarray:
        # merge observation
        if observation_array is None:
            obs_twist = observation_dict['robot_twist']
            merged_obs = np.hstack([observation_dict['laser_scan'], np.array(observation_dict['goal_in_robot_frame']),
                                    np.array([obs_twist.linear.x, obs_twist.linear.y, obs_twist.angular.z])])
        else:
            merged_obs = np.copy(observation_array)

        # normalize
        if self._obs_normalize:
            merged_obs = self._obs_norm_func(merged_obs)

        vel = self._model.predict(merged_obs, deterministic=True)[0]
        return np.array(vel)

    def wait_for_agent(self):
        return True

    def reset(self):
        pass


def load_drl_models_from_config(paths: dict) -> Dict[str, DrlAgent]:
    drl_models = dict()
    with open(paths['all_in_one_parameters'], 'r') as model_json:
        config_data = json.load(model_json)

    assert config_data is not None, "Error: All in one parameter file cannot be found!"

    config_model = config_data['models']

    if 'drl' in config_model:
        agent_dir_names = config_model['drl']
        drl_agent_names = config_model['drl_names']

        for i, agent_dir, in enumerate(agent_dir_names):
            drl_model = DrlAgent(os.path.join(paths['drl_agents'], agent_dir), config_model['drl_names'][i])
            drl_models[drl_agent_names[i]] = drl_model

    return drl_models
