#!/usr/bin/env python
import numpy as np
import rospy
import rospkg

from os import path

import rl_agent.encoder.rosnav_rosnav as r_rosnav
import rl_agent.encoder.navrep_rosnav as n_rosnav
from rl_agent.encoder.guldenring_guldenring import (
    JackalGuldenringEncoder,
    TurtleBot3GuldenringEncoder,
)

import rl_agent.encoder.guldenring_guldenring_pretrained as gring
from rl_agent.encoder.navrep_navrep import NavrepPretrainedEncoder

from rl_agent.base_agent_wrapper import BaseDRLAgent

robot_model = rospy.get_param("model")
""" TEMPORARY GLOBAL CONSTANTS """
NS_PREFIX = ""
TRAINED_MODELS_DIR = lambda env: path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"), "agents", env
)
DEFAULT_ACTION_SPACE = path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    f"default_settings_{robot_model}.yaml",
)

encoders = {
    "rosnav_rosnav": {
        "jackal": r_rosnav.JackalEncoder,
        "ridgeback": r_rosnav.RidgebackEncoder,
        "agv-ota": r_rosnav.AgvEncoder,
        "turtlebot3_burger": r_rosnav.TurtleBot3Encoder,
    },
    "navrep_rosnav": {
        "jackal": n_rosnav.JackalEncoder,
        "ridgeback": n_rosnav.RidgebackEncoder,
        "agv-ota": n_rosnav.AgvEncoder,
        "turtlebot3_burger": n_rosnav.TurtleBot3Encoder,
    },
    "guldenring_guldenring": {
        "turtlebot3_burger": TurtleBot3GuldenringEncoder,
        "jackal": JackalGuldenringEncoder,
    },
    "guldenring_guldenring_pretrained": {
        "agv-ota": gring.GuldenringPretrainedEncoder,
        "turtlebot3_burger": gring.TurtleBot3Encoder,
        "jackal": gring.JackalEncoder,
    },
    "navrep_navrep": {"rto": NavrepPretrainedEncoder},
}


class DeploymentDRLAgent(BaseDRLAgent):
    def __init__(
        self,
        trainings_environment: str,
        model_type: str = "rosnav",
        robot_type: str = "rosnav",
        agent_name: str = "turtlebot3_burger",
        ns: str = None,
        robot_name: str = None,
        action_space_path: str = DEFAULT_ACTION_SPACE,
        *args,
        **kwargs,
    ) -> None:
        """
        Initialization procedure for the DRL agent node.

        Args:
            agent_name (str):
                Agent name (directory has to be of the same name)
            robot_name (str, optional):
                Robot specific ROS namespace extension. Defaults to None.
            ns (str, optional):
                Simulation specific ROS namespace. Defaults to None.
            action_space_path (str, optional):
                Path to yaml file containing action space settings.
                Defaults to DEFAULT_ACTION_SPACE.
        """
        assert encoders[trainings_environment + "_" + model_type][
            robot_type
        ], f"Encoder {robot_type} not available"

        self._is_train_mode = rospy.get_param("/train_mode")
        if not self._is_train_mode:
            rospy.init_node("DRL_local_planner", anonymous=True)

        self._name = agent_name

        hyperparameter_path = path.join(
            TRAINED_MODELS_DIR(trainings_environment),
            self._name,
            "hyperparameters.json",
        )

        super().__init__(
            ns,
            robot_name,
            hyperparameter_path,
            action_space_path,
        )

        self.encoder = encoders[trainings_environment + "_" + model_type][
            robot_type
        ](
            agent_name,
            TRAINED_MODELS_DIR(trainings_environment),
            self._hyperparams,
        )

        self._setup_agent()

        # time period for a valid action
        self._action_period = rospy.get_param("/action_frequency", default=10)
        self._last_action = np.array([0, 0, 0])

        self.STAND_STILL_ACTION = np.array([0, 0, 0])

    def _setup_agent(self) -> None:
        self._agent = self.encoder._agent

    def run(self) -> None:
        """
            Loop for running the agent until ROS is shutdown.
        
            Note:
                Calls the 'step_world'-service for fast-forwarding the \
                simulation time in training mode. The simulation is forwarded \
                by action_frequency seconds. Otherwise, communicates with \
                the ActionPublisher node in order to comply with the specified \
                action publishing rate.
        """
        rate = rospy.Rate(self._action_period)

        while not rospy.is_shutdown():
            goal_reached = rospy.get_param("/bool_goal_reached", default=False)
            if not goal_reached:
                obs = self.get_observations(last_action=self._last_action)

                encoded_obs = self.encoder.get_observation(obs)
                encoded_action = self.encoder.get_action(
                    self.get_action(encoded_obs)
                )

                self.publish_action(encoded_action)
                self._last_action = encoded_action

            rate.sleep()


def main() -> None:
    # TODO load from args if no params
    trainings_environment = rospy.get_param("trainings_environment")
    model_type = rospy.get_param("network_type")
    robot_type = rospy.get_param("model")
    agent_name = rospy.get_param("agent_name")

    AGENT = DeploymentDRLAgent(
        trainings_environment=trainings_environment,
        model_type=model_type,
        robot_type=robot_type,
        agent_name=agent_name,
        ns=NS_PREFIX,
    )

    try:
        AGENT.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
