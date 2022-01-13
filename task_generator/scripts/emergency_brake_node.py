#! /usr/bin/env python3
# see as resource for goal position http://wiki.ros.org/move_base
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan

"""
This node inspects the scanner messages for any obstacles in robots proximity zones
and slows down, makes a full stop accordingly.
"""


class emergency_node:
    def __init__(self):
        """
        __init__ [The node expects the commanded velocity to be published on /nav_vel, then redirects to /cmd_vel]

        """
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.nav_vel_sub = rospy.Subscriber("/nav_vel", Twist, self.nav_vel_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)

    def scan_callback(self, data):
        """
        scan_callback [saves scan message for later use]

        Args:
            data ([LaserScan]): [scan data]
        """
        self.curr_laser_data = data

    def nav_vel_callback(self, data):
        """
        nav_vel_callback [Checks for obstacles near the robot, adapts the commanded velocity]

        Args:
            data ([Twist]): [The desired velocity command]
        """

        scan_data = self.curr_laser_data
        range_min = scan_data.range_min
        ranges = np.asarray(scan_data.ranges)

        ## Checking for obstacles in the restricted zone if any then stop moving
        proximity_ranges = (ranges <= 0.5) & (ranges >= range_min)
        res = np.any(proximity_ranges)
        if res:
            self.cmd_vel_pub.publish(Twist())
            return

        ## Checking for any obstacles in the hazard zone and slowing down accordingly
        proximity_ranges = (ranges <= 1.0) & (ranges >= range_min)
        res = np.any(proximity_ranges)
        if res:
            data.linear.x *= 0.5
            self.cmd_vel_pub.publish(data)
            return
        self.cmd_vel_pub.publish(data)


def main():
    rospy.init_node("emergency-break", anonymous=True)
    node = emergency_node()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
