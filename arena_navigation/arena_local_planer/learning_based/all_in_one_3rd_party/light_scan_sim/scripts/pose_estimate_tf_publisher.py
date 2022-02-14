#!/usr/bin/env python  
# Publishes anything recieved on /initialpose as a TF /initialpose parent /map

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import tf

pose = Pose()

# Update the global pose
def callback_pose(data):
    global pose
    pose = data.pose.pose;

# Publish rviz /initialpose as a tf
if __name__ == '__main__':

    rospy.init_node('pose_estimate_tf_pub')
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, callback_pose)
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        broadcaster.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                                  (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                                  rospy.Time.now(),
                                  "/initialpose",
                                  "/map")
        rate.sleep()
