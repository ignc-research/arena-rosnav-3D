import json
import os

import numpy as np
import rospkg

import rospy
from AIO_local_planners.arena_ros.arena_ros_agent import ArenaRosAgent
from AIO_local_planners.base_local_planner.base_local_planner_all_in_one_interface import BaseLocalPlannerAgent
# from AIO_local_planners.rlca.rlca_agent import RLCAAgent
from AIO_local_planners.rosnav_drl.drl_agent import DrlAgent
from arena_navigation.arena_local_planer.learning_based.all_in_one_planner.scripts.AIO_local_planners.move_base_agent.move_base_agent import \
    MoveBaseAgent


class LocalPlannerManager:

    def __init__(self, paths: dict):
        # set up model parameters
        self._setup_model_configuration(paths)

    def get_numb_models(self) -> int:
        return len(self._models)

    def get_model_names(self) -> [str]:
        return self._model_names

    def get_required_observations(self) -> dict:
        return self._required_observations

    def wait_for_agents(self, sim_step: callable):
        # wait till all agents are ready
        all_ready = False
        while not all_ready:
            all_ready = True
            for i in self._models:
                all_ready = all_ready and i.wait_for_agent()
            if not all_ready:
                if sim_step is not None:
                    sim_step()
                rospy.loginfo("AIO Planner: Not all local planners ready yet! Wait...")

    def execute_local_planner(self, model_numb: int, obs_dict: dict) -> np.array:
        action_model = np.array(self._models[model_numb].get_next_action(obs_dict))
        return action_model

    def reset_planners(self):
        for agent in self._models:
            agent.reset()

    def close_planners(self):
        for m in self._models:
            m.close()

    def _setup_model_configuration(self, paths: dict):
        config_path = paths['all_in_one_parameters']
        models = []
        with open(config_path, 'r') as model_json:
            config_data = json.load(model_json)

        assert config_data is not None, "Error: All in one parameter file cannot be found!"

        config_model = config_data['models']

        # # set up rlca agent
        # if 'rlca' in config_model and config_model['rlca']:
        #     rlca_model = RLCAAgent()
        #     models.append(rlca_model)

        # set up arena_ros agent
        if 'arena_ros' in config_model and config_model['arena_ros']:
            arena_ros_model = ArenaRosAgent()
            models.append(arena_ros_model)

        # set up drl agents
        if 'drl' in config_model:
            agent_dir_names = config_model['drl']

            for i, agent_dir, in enumerate(agent_dir_names):
                save_path = os.path.join(paths['drl_agents'], agent_dir)

                drl_model = DrlAgent(save_path, config_model['drl_names'][i])
                models.append(drl_model)

            self._drl_agent_aliases = config_model['drl_names']

        # set up model based agents based on the all_in_one_base_local_planner package
        base_dir_agent = ''
        if 'model' in paths:
            base_dir_agent = paths['model']

        if 'model_based' in config_model:
            model_based_names = config_model['model_based_names']

            base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
            for i, model_based_config_path in enumerate(config_model['model_based']):
                # case 1: Load config file from agent folder
                model_based_config_path_full = os.path.join(base_dir_agent, model_based_config_path)
                if not os.path.exists(model_based_config_path_full):
                    # case 2: Load config file from config folder
                    model_based_config_path_full = os.path.join(base_dir, 'configs', 'all_in_one_hyperparameters',
                                                                'base_local_planner_parameters',
                                                                model_based_config_path)
                models.append(BaseLocalPlannerAgent(model_based_names[i], model_based_config_path_full))

        if 'move_base_agent' in config_model:
            move_base_agents_names = config_model['move_base_agent']

            for name in move_base_agents_names:
                models.append(MoveBaseAgent(name))

        self._models = models

        # collect necessary observations
        self._required_observations = dict()
        for m in self._models:
            self._required_observations.update(m.get_observation_info())

        # collect model names
        self._model_names = [model.get_name() for model in self._models]
