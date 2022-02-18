#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from numpy import nan, math


class ScanMapper:
    """
    A class that switches the laserscan representations from the /scan topic for different navigation algorithms, see more in the mep_scan.launch

    Parameters: angle_min, angle_max, increment from roslaunch
    Subscribed topics: /scan
    Published topics: /scan_mapped

    """

    def __init__(self):

        # Subscribers (callback functions are triggered on incoming data and written to 'data' as defined by ROS)
        self._robot_state_sub = rospy.Subscriber(
            "/base_scan", LaserScan, self.callback_change_laserscan
        )

        # Publishers
        self._new_scan_pub = rospy.Publisher("/scan_new", LaserScan, queue_size=1)

    def callback_change_laserscan(self, data):
        #print("test")

        ### TB3 Clip scan ranges to 3.5m
        # scan_list = list(data.ranges)
        # for i, element in enumerate(scan_list):
        #     if element > 3.5 or element == 0 
        #         scan_list[i] = 3.5
        # data.ranges = tuple(scan_list)
        #print(data.ranges)

        ### RTO Clip scan ranges to 15m YOUBOT 5.6m
        scan_list = list(data.ranges)
        for i, element in enumerate(scan_list):
            if math.isnan(element) == True or math.isinf(element)== True:
                scan_list[i] = 4
        data.ranges = tuple(scan_list)
        #print(scan_list[1])



        intensity_list = list(data.intensities)
        for i, element in enumerate(intensity_list):
            #if element == 0:
            intensity_list[i] = 0
        data.intensities = tuple(intensity_list)

        ### Map to other scan represenations
        #data.angle_min = rospy.get_param("angle_min")
        #data.angle_max = rospy.get_param("angle_max")
        #data.angle_increment = rospy.get_param("increment")
        self._new_scan_pub.publish(data)


if __name__ == "__main__":
    rospy.init_node("add_offset", anonymous=True, disable_signals=False)

    scanMapper = ScanMapper()
    rospy.spin()
