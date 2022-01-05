#!/usr/bin/env python
import os
import numpy as np
import pickle
import rospy
import rospkg
import sys

from stable_baselines3 import PPO

from rospy.exceptions import ROSException
from std_msgs.msg import Bool

from rl_agent.base_agent_wrapper import BaseDRLAgent
#!/usr/bin/env python
import os
import numpy as np
import pickle
import rospy
import rospkg
import sys

from rospy.exceptions import ROSException
from std_msgs.msg import Bool

from rl_agent.base_agent_wrapper import BaseDRLAgent

sys.modules["arena_navigation.arena_local_planner"] = sys.modules["arena_navigation.arena_local_planer"]

robot_model = rospy.get_param("model")
""" TEMPORARY GLOBAL CONSTANTS """
NS_PREFIX = ""
TRAINED_MODELS_DIR = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"), "agents"
)
DEFAULT_ACTION_SPACE = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    f"default_settings_{robot_model}.yaml",
)


class DeploymentDRLAgent(BaseDRLAgent):
    def __init__(
        self,
        encoder,
        agent_name: str,
        ns: str = None,
        robot_name: str = None,
        model_name: str = None,
        vecnorm_name: str = None,
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
        self.encoder = encoder
        
        self._is_train_mode = rospy.get_param("/train_mode")
        if not self._is_train_mode:
            rospy.init_node("DRL_local_planner", anonymous=True)

        self._name = agent_name

        hyperparameter_path = os.path.join(
            TRAINED_MODELS_DIR, self._name, "hyperparameters.json"
        )

        super().__init__(
            ns,
            robot_name,
            hyperparameter_path,
            action_space_path,
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
        self._action_infered = False
        self._last_action = np.array([0, 0])

        self.STAND_STILL_ACTION = np.array([0, 0])

    def _setup_agent(self, model_name: str, vecnorm_name: str) -> None:
        """Loads the trained policy and when required the VecNormalize object."""
        model_file = os.path.join(
            TRAINED_MODELS_DIR, self.model_name
        )
        vecnorm_file = os.path.join(
            TRAINED_MODELS_DIR, self.vecnorm_name
        )

        assert os.path.isfile(
            model_file
        ), f"Compressed model cannot be found at {model_file}!"
        self._agent = PPO.load(model_file).policy

        if self._hyperparams["normalize"]:
            assert os.path.isfile(
                vecnorm_file
            ), f"VecNormalize file cannot be found at {vecnorm_file}!"

            with open(vecnorm_file, "rb") as file_handler:
                vec_normalize = pickle.load(file_handler)
            self._obs_norm_func = vec_normalize.normalize_obs

    def run(self) -> None:
        """Loop for running the agent until ROS is shutdown.
        
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

                # obs = np.where(obs == np.inf, 3.5, obs)
                # print(obs[360:])
                self._last_action = self.get_action(obs)
                self._action_infered = True

    def callback_publish_action(self):
        if self._action_infered:
            self.publish_action(self._last_action)
            # reset flag
            self._action_infered = False
        else:
            rospy.logdebug("No action received during recent action horizon.")
            self.publish_action(self.STAND_STILL_ACTION)

def main(agent_name: str) -> None:
    AGENT = DeploymentDRLAgent(agent_name=agent_name, ns=NS_PREFIX)

    try:
        AGENT.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    # AGENT_NAME = "pretrained_nonorm_tb3"
    AGENT_NAME = "rule_00"
    main(agent_name=AGENT_NAME)
