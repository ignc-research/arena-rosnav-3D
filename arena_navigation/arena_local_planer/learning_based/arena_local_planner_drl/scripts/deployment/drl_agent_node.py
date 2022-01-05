#!/usr/bin/env python
import numpy as np
import rospy
import rospkg
import sys

from os import path

from rl_agent.encoder.rosnav_rosnav import *
from rl_agent.encoder.navrep_rosnav import *
from rl_agent.base_agent_wrapper import BaseDRLAgent


sys.modules["arena_navigation.arena_local_planner"] = sys.modules["arena_navigation.arena_local_planer"]

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
        "jackal": JackalRosnavEncoder,
        "ridgeback": RidgebackRosnavEncoder,
        "agv": AgvRosnavEncoder,
        "tb3": TurtleBot3RosnavEncoder
    },
    "navrep_rosnav": {
        "jackal": JackalNavrepEncoder,
        "ridgeback": RidgebackNavrepEncoder,
        "agv": AgvNavrepEncoder,
        "tb3": TurtleBot3NavrepEncoder
    }
}

class DeploymentDRLAgent(BaseDRLAgent):
    def __init__(
        self,
        encoder: str,
        agent_name: str,
        environment: str = "rosnav",
        model_env: str = "rosnav",
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
        assert encoders[model_env + "_" + environment][encoder], f"Encoder {encoder} not available"

        self._is_train_mode = rospy.get_param("/train_mode")
        if not self._is_train_mode:
            rospy.init_node("DRL_local_planner", anonymous=True)

        self._name = agent_name

        hyperparameter_path = path.join(
            TRAINED_MODELS_DIR(model_env), self._name, "hyperparameters.json"
        )

        super().__init__(
            ns,
            robot_name,
            hyperparameter_path,
            action_space_path,
        )
        
        self.encoder = encoders[model_env + "_" + environment][encoder](
            agent_name, TRAINED_MODELS_DIR(model_env), self._hyperparams
        )
        
        self._setup_agent()
        # if self._is_train_mode:
        #     # step world to fast forward simulation time
        #     self._service_name_step = f"{self._ns}step_world"
        #     # self._sim_step_client = rospy.ServiceProxy(
        #     #     self._service_name_step, StepWorld
        #     # )

        # time period for a valid action
        self._action_period = rospy.Duration(
            1 / rospy.get_param("/action_frequency", default=10)
        )  # in seconds
        self._action_inferred = False
        self._curr_action, self._last_action = np.array([0, 0]), np.array(
            [0, 0]
        )

        self.STAND_STILL_ACTION = np.array([0, 0])

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
        rospy.Timer(self._action_period, self.callback_publish_action)

        while not rospy.is_shutdown():
            goal_reached = rospy.get_param("/bool_goal_reached", default=False)
            if not goal_reached:
                obs = self.get_observations()[0]

                encoded_obs = self.encoder.get_observation(obs)
                encoded_action = self.encoder.get_action(self.get_action(encoded_obs))

                self._last_action = self._curr_action
                self._curr_action = encoded_action

                self._action_inferred = True

    def callback_publish_action(self):
        if self._action_infered:
            self.publish_action(self._last_action)
            # reset flag
            self._action_inferred = False
        else:
            rospy.logdebug(
                "[DRL_NODE]: No action inferred during most recent action cycle."
            )
            self.publish_action(self.STAND_STILL_ACTION)

def main(env: str, encoder: str, model_env: str, agent_name: str) -> None:
    AGENT = DeploymentDRLAgent(
        encoder, 
        agent_name=agent_name, 
        environment=env,
        model_env=model_env, 
        ns=NS_PREFIX
    )

    try:
        AGENT.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    AGENT_NAME = sys.argv[1]
    environment = sys.argv[2]
    encoder = sys.argv[3]
    model_env = sys.argv[4]
    main(environment, encoder, model_env, agent_name=AGENT_NAME)
