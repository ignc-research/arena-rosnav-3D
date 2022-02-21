import numpy as np
import rospy
from geometry_msgs.msg import Twist

from arena_navigation.arena_local_planer.learning_based.all_in_one_planner.scripts.AIO_local_planners.model_base_class import \
    ModelBase


class MoveBaseAgent(ModelBase):

    def __init__(self, name: str):
        super().__init__(dict(), name)
        self._last_cmd_vel = Twist()

        self._robot_state_sub = rospy.Subscriber('cmd_vel_move_base_' + name, Twist, self._callback_cmd_vel, tcp_nodelay=True)

    def get_next_action(self, observation_dict: dict) -> np.ndarray:
        return np.array([self._last_cmd_vel.linear.x, self._last_cmd_vel.linear.y, self._last_cmd_vel.angular.z])

    def wait_for_agent(self):
        return True

    def reset(self):
        pass

    def _callback_cmd_vel(self, msg_twist):
        self._last_cmd_vel = msg_twist