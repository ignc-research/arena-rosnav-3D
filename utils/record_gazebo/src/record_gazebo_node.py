import rospy
import json
import sys
import rospkg

import numpy as np

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest

from geometry_msgs.msg import Twist


""" 
    This ROS node creates Camera Nodes in Gazebo based on a given declaration file.

    The Cameras start recording as soon as /cmd_vel is published and stop recording when the program is terminated.
    After terminating videos can be created with the create_video.py skript
"""


rospack = rospkg.RosPack()
package_path = rospack.get_path("record_gazebo")


def spawn_camera_models(camera_setup, data):
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for camera in camera_setup:
        camera_model_path = package_path + "/resource/camera.sdf" 

        pose = data[camera]["pose"]

        request = SpawnModelRequest()
        request.model_name = camera
        request.initial_pose.position.x = pose["x"]
        request.initial_pose.position.y = pose["y"]
        request.initial_pose.position.z = pose["z"]

        quaternion = euler_to_quaternion(pose["yaw"], pose["pitch"], pose["roll"])

        request.initial_pose.orientation.x = quaternion[0]
        request.initial_pose.orientation.y = quaternion[1]
        request.initial_pose.orientation.z = quaternion[2]
        request.initial_pose.orientation.w = quaternion[3]

        request.model_xml = open(camera_model_path).read()
    
        spawn_model_service(request)


def get_camera_setup_from_name(camera_setup_name):
    json_file = open(package_path + "/resource/camera_setup.json", "r")
    data = json.load(json_file)

    if not camera_setup_name in data.keys():
        print("Camera setup does not exist")
        sys.exit(1)

    return data[camera_setup_name]


def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]


if __name__ == "__main__":
    rospy.init_node("record_gazebo_node")

    rospy.wait_for_message("/cmd_vel", Twist)

    camera_setup_name = rospy.get_param("/camera_setup", default="default")

    camera_setup = get_camera_setup_from_name(camera_setup_name)

    spawn_camera_models(camera_setup["setup"], camera_setup)



