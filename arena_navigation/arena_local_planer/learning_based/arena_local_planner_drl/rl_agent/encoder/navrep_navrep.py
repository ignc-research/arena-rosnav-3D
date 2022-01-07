from os import path
import sys
import numpy as np

from navrep.tools.custom_policy import Custom1DPolicy
from stable_baselines.ppo2 import PPO2

from rl_agent.encoder import BaseEncoder

sys.modules["custom_policy"] = sys.modules["navrep.tools.custom_policy"]

class E2E1DEncoder(BaseEncoder):
    def __init__(self, agent_name: str, model_dir: str, hyperparams):
        model_path = path.join(model_dir, agent_name + ".zip")

        assert path.isfile(
            model_path
        ), f"Compressed model cannot be found at {model_path}!"

        self._agent = self._load_model(model_path)
        if hyperparams["normalize"]:
            self._obs_norm_func = self._load_vecnorm()

    def _load_model(self, model_path: str):
        return PPO2.load(model_path, policy=Custom1DPolicy)

    def _load_vecnorm(self):
        return lambda obs: obs

    def get_observation(self, obs):
        scan, _, _, observation = obs
        subgoal, robot_pose, robot_vel = observation 

        model_input = np.hstack(
            [    
                scan, 
                [subgoal.x - robot_pose.x, subgoal.y - robot_pose.y], 
                [robot_vel.linear.x, robot_vel.linear.y, 0]
            ]
        )

        print(model_input)

        return model_input.reshape(len(model_input), 1)

    def get_action(self, action):
        """
            Encodes the action produced by the nn
            Should always return an array of size 3 with entries
            [x_vel, y_vel, ang_vel]
        """
        assert len(action) == 2, f"Expected an action of size 2 but received {len(action)}: {action}"
        

        x_vel, y_vel = action
        return [x_vel, y_vel]

# """
#     Jackal
#     N: 720 
#     offset: -3/4 * pi
#     action: [x_vel, ang_vel]
# """
# class JackalE2E1DEncoder(E2E1DEncoder):
#     pass

# """
#     Turtlebot3
#     N: 360
#     offset: 0
#     action: [x_vel, ang_vel]
# """
# class TurtleBot3E2E1DEncoder(E2E1DEncoder):
#     pass

# """
#     AGV
#     N: 720
#     offset: -pi
#     action: [x_vel, ang_vel]
# """
# class AgvE2E1DEncoder(E2E1DEncoder):
#     pass

# """
#     Ridgeback
#     N: 720
#     offset: -3/4 * pi
#     action: [x_vel, y_vel, ang_vel]
# """
# class RidgebackE2E1DEncoder(E2E1DEncoder):
#     def get_action(action):
#         assert len(action) == 3, f"Expected an action of size 3 but received: {action}"

#         return action

class NavrepPepperE2E1dEncoder(E2E1DEncoder):
    pass