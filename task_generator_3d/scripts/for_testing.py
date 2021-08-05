#! /usr/bin/env python
# from math import ceil, sqrt
from nav_msgs.msg import OccupancyGrid, Path
import math
import yaml
import os
import threading
import rospy
import tf
# from flatland_msgs.srv import MoveModel, MoveModelRequest, SpawnModelRequest, SpawnModel
# from flatland_msgs.srv import StepWorld
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, PoseStamped, Pose, Point
from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState


# from .utils import generate_freespace_indices, get_random_pos_on_map


def main():
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    rospy.wait_for_service('/gazebo/set_model_state')
    _srv_move_model = rospy.ServiceProxy(
        '/gazebo/set_model_state', SetModelState)
    _srv_spawn_model = rospy.ServiceProxy(
        '/gazebo/spawn_urdf_model', SpawnModel)
    model_name = 'turtlebot3_burger'
    model_xml = open(
        '/home/jacek/turtlebot3_burger.urdf', 'r').read()
    robot_namespace = '/'
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0

    # initial_pose.position.z = 1
    reference_frame = 'world'
    _srv_spawn_model(model_name, model_xml, robot_namespace,
                     initial_pose, reference_frame)
    # Try to move the robot
    #
    #
    # rospy.sleep(5)
    # new_state = ModelState()
    # new_state.model_name = 'myrobot'
    # random_pos = Pose2D(-3, -3, 20)
    # pos_3d = Pose()
    # quaternion = tf.transformations.quaternion_from_euler(
    #     0, 0, random_pos.theta)
    # pos_3d.orientation.x = quaternion[1]
    # pos_3d.orientation.w = quaternion[0]
    # pos_3d.orientation.y = quaternion[2]
    # pos_3d.orientation.z = quaternion[3]
    # new_state.pose = pos_3d
    # _srv_move_model(new_state)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
