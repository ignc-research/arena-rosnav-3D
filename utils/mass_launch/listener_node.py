#!/usr/bin/env python3

import rospy, time, subprocess
from std_msgs.msg import Bool

class listen:
    def __init__(self):
        rospy.init_node("listener_for_scenario_end", anonymous=True)
        time.sleep(10)
        rospy.wait_for_message('End_of_scenario', Bool)

        # terminate gazebo
        subprocess.Popen(
            "killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient",
            shell=True,
        )

        # terminate rosnodes
        subprocess.Popen("rosnode kill --all", shell=True)

        rospy.signal_shutdown("End of scenario")

if __name__ == "__main__":
    listen = listen()
    rospy.spin()
