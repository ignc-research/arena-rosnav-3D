#!/usr/bin/env python

import os
import csv
import rospy
import rosnode
import roslaunch
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import ChannelFloat32

from collision_check import GOAL_CHECKING_THRESHOLD


class MetricMonitor():
    def __init__(self):
        self.reset()

    def reset(self):
        self.curr_duration = 0.0
        self.curr_distance = 0.0
        self.trial_running = True
        self.prev_distance = 0.0
        self.prev_duration = 0.0
        init_x = rospy.get_param("/quadrotor_simulator_so3/simulator/init_state_x", -18.0)
        init_y = rospy.get_param("/quadrotor_simulator_so3/simulator/init_state_y", 0.0)
        self.prev_goal = np.array([init_x, init_y])
        self.curr_goal = None
        self.goal_reaching_performance = []

    def update_collision_status(self, msg):
        duration, distance = msg.values
        if duration == -1.0:
            self.trial_running = False
        else:
            self.curr_duration = duration
            self.curr_distance = distance

    def update_goal(self, msg):
        position = msg.pose.position
        goal = np.array([position.x, position.y])
        if self.curr_goal is None:
            self.curr_goal = goal
        else:
            goal_distance = np.linalg.norm(self.curr_goal - self.prev_goal) - GOAL_CHECKING_THRESHOLD
            traversal_dist = self.curr_distance - self.prev_distance
            traversal_time = self.curr_duration - self.prev_duration
            self.prev_distance = self.curr_distance
            self.prev_duration = self.curr_duration
            self.prev_goal = self.curr_goal
            self.curr_goal = goal
            self.goal_reaching_performance.append([goal_distance, traversal_time, traversal_dist, 1])


if __name__ == "__main__":
    c_num = 400
    p_num = 400
    random_forest = True
    use_LfH = False

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    monitor = MetricMonitor()
    rospy.init_node("duration_listener", anonymous=True)
    rospy.Subscriber("collision_status", ChannelFloat32, monitor.update_collision_status)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, monitor.update_goal)

    for i in range(0, 14):
        monitor.reset()

        args_list = ["run_in_sim_test.launch", "c_num:=" + str(c_num), "seed:=" + str(i),
                     "bspline_topic:=" + str("/planning/bspline_truth" if use_LfH else "/planning/bspline"),
                     "use_lfh:=" + str("true" if use_LfH else "false"),
                     "random_forest:=" + str("true" if random_forest else "false"),
                     ]
        lifelong_args = args_list[1:]
        launch_files = [(roslaunch.rlutil.resolve_launch_arguments(args_list)[0], lifelong_args)]

        # launch the launch file
        parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        parent.start()

        trial_start = rospy.get_time()

        while monitor.trial_running:
            if rospy.get_time() - trial_start > 60.0 * 20:
                break

        parent.shutdown()

        if not monitor.trial_running:
            collision_fname = "{}_collision.csv".format("LfH" if use_LfH else "ego")
            collision_fname_exists = os.path.exists(collision_fname)
            with open(collision_fname, "a") as f:
                csvwriter = csv.writer(f)
                if not collision_fname_exists:
                    csvwriter.writerow(["duration", "distance"])
                csvwriter.writerow([monitor.curr_duration, monitor.curr_distance])

        if not monitor.trial_running:
            goal_distance = np.linalg.norm(monitor.curr_goal - monitor.prev_goal) - GOAL_CHECKING_THRESHOLD
            traversal_dist = monitor.curr_distance - monitor.prev_distance
            traversal_time = monitor.curr_duration - monitor.prev_duration
            monitor.goal_reaching_performance.append([goal_distance, traversal_time, traversal_dist, 0])
        traversal_fname = "{}_traversal.csv".format("LfH" if use_LfH else "ego")
        traversal_fname_exists = os.path.exists(traversal_fname)
        with open(traversal_fname, "a") as f:
            csvwriter = csv.writer(f)
            if not traversal_fname_exists:
                csvwriter.writerow(["goal_distance", "traversal_time", "traversal_distance", "success"])
            csvwriter.writerows(monitor.goal_reaching_performance)

        print "Finished %d" % i
