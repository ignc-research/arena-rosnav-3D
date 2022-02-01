#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class velocity_redirect:
    def __init__(self):
        rospy.init_node("listener", anonymous=True)
        self.topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        rospy.Subscriber("/cmd_vel", Twist, self.vel_callback)
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=10)

    def vel_callback(self, data):
        self.pub.publish(data)


if __name__ == "__main__":
    redirect = velocity_redirect()
    rospy.spin()
