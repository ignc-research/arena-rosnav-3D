#! /usr/bin/env python3
# see as resource for goal position http://wiki.ros.org/move_base
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped, Twist
from actionlib_msgs.msg import GoalID

global stopped, current_goal


class emergency_node:
    def __init__(self):
        self.current_goal = PoseStamped()
        self.stopped = False
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # self.goal_sub = rospy.Subscriber(
        #     "/move_base_simple/goal", PoseStamped, self.goal_callback
        # )
        # self.pub_mvb_goal = rospy.Publisher(
        #     "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        # )
        # self.pub_goal = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        # self.pub_mvb_goal_cancel = rospy.Publisher(
        #     "/move_base/cancel", GoalID, queue_size=1, latch=True
        # )
        self.nav_vel_sub = rospy.Subscriber("/nav_vel", Twist, self.nav_vel_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)

    def scan_callback(self, data):
        """When laser spots any obstale in its proximity (<= 1m) navigation goal gets cancelled. When obstacles  is cleared, the navigation is resumed."""
        self.curr_laser_data = data
        # range_min = data.range_min
        # ranges = np.asarray(data.ranges)

        # proximity_ranges = (ranges <= 0.8) & (ranges >= range_min)
        # res = np.any(proximity_ranges)
        # if (
        #     res
        # ):  # Some obstacle near the robot was found, cancel the current goal and save it in order to resume the navigation later.
        #     self.pub_mvb_goal_cancel.publish(
        #         GoalID(stamp=rospy.Time.from_sec(0.0), id="")
        #     )
        #     print("Stopping the robot")
        #     self.stopped = True
        # elif self.stopped:
        #     print("Resume")
        #     self.current_goal.header.stamp = rospy.Time.now()
        #     # self.pub_goal.publish(self.current_goal)
        #     self.pub_mvb_goal.publish(self.current_goal)
        #     self.stopped = False

    def nav_vel_callback(self, data):
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

    def goal_callback(self, msg):
        self.current_goal = msg


def main():
    rospy.init_node("emergency-break", anonymous=True)
    node = emergency_node()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
