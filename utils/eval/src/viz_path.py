#! /usr/bin/env python
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Vector3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import *
import tf2_ros

# viz
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

color_dict = {
    "red" : ColorRGBA(r=1.0, a=1.0),
    "green" : ColorRGBA(g=1.0, a=1.0),
    "blue" : ColorRGBA(b=1.0, a=1.0),
    "light blue" : ColorRGBA(r=.29, g=.84, b=1.0, a=1.0),
    "yellow" : ColorRGBA(r=1.0, g=1.0, a=1.0),
    "pink" : ColorRGBA(r=1.0, b=1.0, a=1.0),
    "purple" : ColorRGBA(r=.3, b=.44, a=1.0),
    "orange" : ColorRGBA(r=255, g=140, b=0, a=1.0),
}

class VizualizePath():
    def __init__(self, color = "red"):
        self.color = color
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # subs
        rospy.Subscriber(
            "/odom",
            Odometry,
            self.pose_callback,
            tcp_nodelay=True,
        )
        rospy.Subscriber("/scenario_reset", Int16, self.scenario_reset_callback)

        # pubs
        self.pub_pose_marker = rospy.Publisher('/viz_agent_pose',Marker,queue_size=1)

        self.num_poses = 0

    def pose_callback(self, odom):
        self.num_poses += 1

        if not rospy.get_param("/real", default=False):
            self.publish_new_position(odom.pose.pose.position, odom.pose.pose.orientation)
            return

        # map -> base_footprint/ base_link tf = robot pose
        try:
            if rospy.get_param("model") == 'turtlebot3_burger':
                child_frame = "base_footprint"
                tf = self.tfBuffer.lookup_transform(
                    "map", child_frame, rospy.Time()
                )
            else:
                child_frame = "base_link"
                tf = self.tfBuffer.lookup_transform(
                    "map", child_frame, rospy.Time()
                )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            print("No map to " + child_frame + " transform!")
            return

        self.publish_new_position(self.tf_to_point(tf), tf.transform.rotation)

    def scenario_reset_callback(self, _):
        self.num_poses = 0
        self.pub_pose_marker.publish(
            VizualizePath.create_reset_marker()
        )

    def publish_new_position(self, pos, orientation):
        if rospy.get_param("/bool_goal_reached", default=False):
            return

        self.pub_pose_marker.publish(
            VizualizePath.create_marker(
                pos, orientation, self.num_poses, self.color
            )
        )
    
    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down path visualizer.")


    @staticmethod
    def create_reset_marker():
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = marker.DELETEALL

        return marker

    @staticmethod
    def tf_to_point(tf):
        point = Point()
        point.x = tf.transform.translation.x
        point.y = tf.transform.translation.y
        point.z = tf.transform.translation.z

        return point

    @staticmethod
    def create_marker(pos, orientation, id, color):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'agent'
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position = pos
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.05, y=0.05, z=0.05)
        marker.color = color_dict[color]
        marker.lifetime = rospy.Duration(0)

        return marker


if __name__ == "__main__":
    rospy.init_node('path_visualizer', anonymous=False)

    print('==================================\nPath visualization started\n==================================')

    rospy.wait_for_message("/cmd_vel", Twist)

    color = rospy.get_param("path_color", "blue")
    visualizer = VizualizePath(color = color)

    rospy.on_shutdown(visualizer.on_shutdown)

    rospy.spin()