#!/usr/bin/env python


from pathlib import Path
import json, six, abc
import rospy, math, time, random
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from threading import Lock
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import Int16
import numpy as np
from tf.transformations import quaternion_from_euler
from .utils import generate_freespace_indices, get_random_pos_on_map

ROBOT_RADIUS = 0.17

class RobotManager:
    """
    A manager class using gazebo provided services to spawn, move and delete Robot. Currently only one robot
    is managed
    """

    def __init__(self, ns, map_):
        # type (str, OccupancyGrid, str, int) -> None
        """[summary]
        Args:
            ns(namespace): if ns == '', we will use global namespace
            map_ (OccupancyGrid): the map info
            robot_yaml_path (str): the file name of the robot yaml file.
        """
        self.ns = ns
        self.update_map(map_)

    def update_map(self, new_map):
        # type (OccupancyGrid) -> None
        self.map = new_map
        self._free_space_indices = generate_freespace_indices(self.map)

    def move_robot(self, pose):
        # type: (Pose) -> None
        """move the robot to a given position
        Args:
            pose (Pose): target postion
        """
        start_pos = ModelState()
        start_pos.model_name = 'turtlebot3'
        start_pos.pose = pose
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(start_pos)

        except rospy.ServiceException:
            print("Move Robot to position failed")

        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 10)
        rospy.sleep(3)
        start_pos = PoseWithCovarianceStamped()
        start_pos.header.frame_id = 'map'
        start_pos.pose.pose = pose # Achtung in Random task hier .pose.pose
        pub.publish(start_pos)

    def publish_goal(self, pose):
        # type: (Pose) -> None
        """
        Publishing goal (x, y, theta)
        :param x x-position of the goal
        :param y y-position of the goal
        :param theta theta-position of the goal
        """
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = pose

        client.send_goal(self.goal)
        wait = client.wait_for_result()
        if not wait: ############################################can be deleted later
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

    def set_start_pos_random(self):
        start_pos = Pose()
        start_pos = get_random_pos_on_map(
            self._free_space_indices, self.map, ROBOT_RADIUS)
        self.move_robot(start_pos)

    def set_start_pos_goal_pos(self, start_pos = None, goal_pos = None, min_dist=1, forbidden_zones = None):
        # type: (Union[Pose2D, None], Union[Pose2D, None], int)
        """set up start position and the goal postion. Path validation checking will be conducted. If it failed, an
        exception will be raised.
        Args:
            start_pos (Union[Pose2D,None], optional): start position. if None, it will be set randomly. Defaults to None.
            goal_pos (Union[Pose2D,None], optional): [description]. if None, it will be set randomly .Defaults to None.
            min_dist (float): minimum distance between start_pos and goal_pos
        Exception:
            Exception("can not generate a path with the given start position and the goal position of the robot")
        """

        def dist(x1, y1, x2, y2):
            return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

        if start_pos is None or goal_pos is None:
            # if any of them need to be random generated, we set a higher threshold,otherwise only try once
            max_try_times = 20
        else:
            max_try_times = 1
        if forbidden_zones == None: forbidden_zones = None # change later
        i_try = 0
        start_pos_ = None
        goal_pos_ = None
      
        while i_try < max_try_times:

            if start_pos is None:
                start_pos_ = get_random_pos_on_map(
                    self._free_space_indices, self.map, ROBOT_RADIUS * 2, forbidden_zones)
            else:
                start_pos_ = start_pos
                
            if goal_pos is None:
                goal_pos_ = get_random_pos_on_map(
                    self._free_space_indices, self.map, ROBOT_RADIUS * 2, forbidden_zones)
            else:
                goal_pos_ = goal_pos
                
            if dist(start_pos_.position.x, start_pos_.position.y, goal_pos_.position.x, goal_pos_.position.y) < min_dist:
                i_try += 1
                continue
            # move the robot to the start pos
            self.move_robot(start_pos_)
            try:
                # publish the goal, if the gobal plath planner can't generate a path, a, exception will be raised.
                self.publish_goal(goal_pos_)
                break
            except rospy.ServiceException:
                i_try += 1
        if i_try == max_try_times:
            # TODO Define specific type of Exception
            raise rospy.ServiceException(
                "can not generate a path with the given start position and the goal position of the robot")
        else:
            return start_pos_, goal_pos_
