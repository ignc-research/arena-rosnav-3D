import numpy as np
import rospy
import rospkg
import os

rospy.set_param("/MARL", True)

from rl_agent.training_agent_wrapper import TrainingDRLAgent
from scripts.deployment.drl_agent_node import DeploymentDRLAgent
from rl_agent.envs.pettingzoo_env import GazeboPettingZooEnv

from nav_msgs.srv import GetMap

DEFAULT_HYPERPARAMETER = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    "hyperparameters",
    "default.json",
)
DEFAULT_ACTION_SPACE = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    "default_settings.yaml",
)


def instantiate_drl_agents(
    num_robots: int = 1,
    ns: str = None,
    robot_name_prefix: str = "robot",
    hyperparameter_path: str = DEFAULT_HYPERPARAMETER,
    action_space_path: str = DEFAULT_ACTION_SPACE,
) -> list:
    return [
        TrainingDRLAgent(
            ns=ns,
            robot_name=robot_name_prefix + str(i + 1),
            hyperparameter_path=hyperparameter_path,
            action_space_path=action_space_path,
        )
        for i in range(num_robots)
    ]


def main():
    rospy.set_param("/MARL", True)
    rospy.init_node(f"USER_NODE", anonymous=True)

    agent_list = instantiate_drl_agents(num_robots=4, ns="sim_1")

    env = GazeboPettingZooEnv(ns="sim_1", agent_list=agent_list)
    obs = env.reset()

    AGENT = DeploymentDRLAgent(
        agent_name="rule_04", ns="sim_1", robot_name="test1"
    )

    agent_names = env.agents
    for _ in range(100000000):
        actions = {agent: AGENT.get_action(obs[agent]) for agent in agent_names}
        obs, rewards, dones, infos = env.step(actions)


if __name__ == "__main__":
    main()
