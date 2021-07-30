import os
from abc import ABC, abstractmethod
from threading import Condition, Lock
from filelock import FileLock

import rospy
import rospkg
import json
import yaml
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose2D
from rospy.exceptions import ROSException

from std_msgs.msg import Bool

from .obstacles_manager import ObstaclesManager
from .robot_manager import RobotManager
from pathlib import Path


class StopReset(Exception):
    """Raised when The Task can not be reset anymore """


class ABSTask(ABC):
    """An abstract class, all tasks must implement reset function.

    """

    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        #self.obstacles_manager = obstacles_manager
        self.robot_manager = robot_manager
        self._service_client_get_map = rospy.ServiceProxy('/static_map', GetMap)
        self._map_lock = Lock()
        rospy.Subscriber('/map', OccupancyGrid, self._update_map)
        # a mutex keep the map is not unchanged during reset task.

    @abstractmethod
    def reset(self):
        """
        a funciton to reset the task. Make sure that _map_lock is used.
        """scenario

    def __init__(self, obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        super().__init__(obstacles_manager, robot_manager)

    def reset(self):
        """[summary]
        """
        with self._map_lock:
            max_fail_times = 3
            fail_times = 0
            while fail_times < max_fail_times:
                try:
                    start_pos, goal_pos = self.robot_manager.set_start_pos_goal_pos()
                    # self.obstacles_manager.reset_pos_obstacles_random(
                    #     forbidden_zones=[
                    #         (start_pos.x,
                    #             start_pos.y,
                    #             self.robot_manager.ROBOT_RADIUS),
                    #         (goal_pos.x,
                    #             goal_pos.y,
                    #             self.robot_manager.ROBOT_RADIUS)])
                    break
                except rospy.ServiceException as e:
                    rospy.logwarn(repr(e))
                    fail_times += 1
            if fail_times == max_fail_times:
                raise Exception("reset error!")


class ManualTask(ABSTask):
    """randomly spawn obstacles and user can mannually set the goal postion of the robot
    """

    def __init__(self,obstacles_manager: ObstaclesManager, robot_manager: RobotManager):
        super().__init__(obstacles_manager, robot_manager)
        # subscribe
        rospy.Subscriber(f'{self.ns}manual_goal', Pose2D, self._set_goal_callback)
        self._goal = Pose2D()
        self._new_goal_received = False
        self._manual_goal_con = Condition()

    def reset(self):
        while True:
            with self._map_lock:
                #self.obstacles_manager.reset_pos_obstacles_random()
                self.robot_manager.set_start_pos_random()
                with self._manual_goal_con:
                    # the user has 60s to set the goal, otherwise all objects will be reset.
                    self._manual_goal_con.wait_for(
                        self._new_goal_received, timeout=60)
                    if not self._new_goal_received:
                        raise Exception(
                            "TimeOut, User does't provide goal position!")
                    else:
                        self._new_goal_received = False
                    try:
                        # in this step, the validation of the path will be checked
                        self.robot_manager.publish_goal(
                            self._goal.x, self._goal.y, self._goal.theta)
                    except Exception as e:
                        rospy.logwarn(repr(e))

    def _set_goal_callback(self, goal: Pose2D):
        with self._manual_goal_con:
            self._goal = goal
            self._new_goal_received = True
        self._manual_goal_con.notify()


def get_predefined_task(ns: str, mode="random", start_stage: int = 1, PATHS: dict = None):

    # TODO extend get_predefined_task(mode="string") such that user can choose between task, if mode is

    # check is it on traininig mode or test mode. if it's on training mode
    # flatland will provide an service called 'step_world' to change the simulation time
    # otherwise it will be bounded to real time.

    # either e.g. ns = 'sim1/' or ns = ''

    # get the map
    
    service_client_get_map = rospy.ServiceProxy('/static_map', GetMap) 
    map_response = service_client_get_map()

    # use rospkg to get the path where the model config yaml file stored
    models_folder_path = rospkg.RosPack().get_path('simulator_setup') #$(find simulator_setup)/maps/$(arg map_file)/map.yaml"
    
    # robot's yaml file is needed to get its radius.
    robot_manager = RobotManager(ns, map_response.map, os.path.join(
        models_folder_path, 'robot', "myrobot.model.yaml"))
    
    obstacles_manager = ObstaclesManager(ns, map_response.map)
    
    # only generate 3 static obstaticles
    # obstacles_manager.register_obstacles(3, os.path.join(
    # models_folder_path, "obstacles", 'random.model.yaml'), 'static')
    # generate 5 static or dynamic obstaticles
    # obstacles_manager.register_random_obstacles(20, 0.4)

    # TODO In the future more Task will be supported and the code unrelated to
    # Tasks will be moved to other classes or functions.
    task = None
    # if mode == "random":
    #     rospy.set_param("/task_mode", "random")
    #     obstacles_manager.register_random_obstacles(20, 0.4)
    #     task = RandomTask(obstacles_manager, robot_manager)
    #     print("random tasks requested")
    if mode == "manual":
        rospy.set_param("/task_mode", "manual")
        #obstacles_manager.register_random_obstacles(20, 0.4)
        task = ManualTask(obstacles_manager, robot_manager)
        print("manual tasks requested")
    # if mode == "staged":
    #     rospy.set_param("/task_mode", "staged")
    #     task = StagedRandomTask(
    #         ns, obstacles_manager, robot_manager, start_stage, PATHS)
    # if mode == "scenario":
    #     rospy.set_param("/task_mode", "scenario")
    #     task = ScenerioTask(obstacles_manager, robot_manager,
    #         PATHS['scenario']) #default="$(find simulator_setup)/scenarios/empty_map.json
    return task
