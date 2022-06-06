#!/usr/bin/env python
#
# file: $ISIP_EXP/tuh_dpath/exp_0074/scripts/decode.py
#
# revision history:
#  20190925 (TE): first version
#
# usage:
#  python decode.py odir mfile data
#
# arguments:
#  odir: the directory where the hypotheses will be stored
#  mfile: input model file
#  data: the input data list to be decoded
#
# This script decodes data using a simple MLP model.
#------------------------------------------------------------------------------

# import modules
#
import sys
import os

# ros:
import rospy
import numpy as np 

# custom define messages:
from sensor_msgs.msg import LaserScan
from barn_msgs.msg import BARN_data
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from stable_baselines3 import PPO
from custom_cnn_full import *
from move_base import MoveBase


#-----------------------------------------------------------------------------
#
# global variables are listed here
#
#-----------------------------------------------------------------------------
#set_seed(SEED1)

# for reproducibility, we seed the rng
#       
policy_kwargs = dict(
    features_extractor_class=CustomCNN,
    features_extractor_kwargs=dict(features_dim=256),
)

#------------------------------------------------------------------------------
#
# the main program starts here
#
#------------------------------------------------------------------------------
class DrlInference:
    # Constructor
    def __init__(self, model=None):
        # initialize data:  
        self.LASER_CLIP = 10
        self.scan = [] 
        self.goal = []
        self.vx = 0
        self.wz = 0
        self.model = None
        
        # parameters:
        self.start = rospy.get_param('~start', True)
        goal_x = rospy.get_param('~goal_x', 0)
        goal_y = rospy.get_param('~goal_y', 10)

        # launch move_base:
        self.goal_position = [goal_x, goal_y, 0]
        self.base_local_planner = "base_local_planner/TrajectoryPlannerROS"
        self.move_base = MoveBase(goal_position=self.goal_position, base_local_planner=self.base_local_planner)
        # make plan: 
        self.move_base.reset_robot_in_odom()
        self.move_base.make_plan()
        self._clear_costmap()
    
        # Might not need this dict in all cases
        custom_objects = {
            "lr_schedule": lambda x: .003,
            "clip_range": lambda x: .02
        }
        # load model:
        if(model == None):
            model_file = rospy.get_param('~model_file', "./model/drl_vo.zip")
            self.model = PPO.load(model_file, custom_objects=custom_objects)
        else:
            self.model = model
        print("Finish loading model.")

        # initialize ROS objects
        self.barn_data_sub = rospy.Subscriber("/barn_data", BARN_data, self.barn_data_callback, queue_size=2, buff_size=2**24)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10, latch=False)
        
    # Callback function for the barn_data subscriber
    def barn_data_callback(self, barn_data_msg):
        self.scan = barn_data_msg.scan
        self.goal = barn_data_msg.goal_cart
        self.move_base.make_plan()  
        cmd_vel = Twist()

        if(self.start): # start navigation  
            # minimum distance:
            scan = np.array(self.scan[-720+240:-240])
            scan = scan[scan!=0]
            if(scan.size!=0):
                min_scan_dist = np.amin(scan)
            else:
                min_scan_dist = 10
        
            # if the goal is close to the robot:
            if(np.linalg.norm(self.goal) <= 0.9):  # goal margin
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0
            elif(min_scan_dist <= 0.45): # obstacle margin
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0.7
            else:
                # ped_map:
                ped_pos = np.zeros(12800)

                # scan_map:
                laser_scan = np.array(self.scan, dtype=np.float32)
                laser_scan[laser_scan > self.LASER_CLIP] = self.LASER_CLIP
                # min-avg pooling:
                scan_avg = np.zeros((20,80))
                for n in range(10):
                    scan_tmp = laser_scan[n*720:(n+1)*720]
                    for i in range(80):
                        scan_avg[2*n, i] = np.min(scan_tmp[i*9:(i+1)*9])
                        scan_avg[2*n+1, i] = np.mean(scan_tmp[i*9:(i+1)*9])
                # stacking: 
                scan_avg = scan_avg.reshape(1600)
                scan_avg_map = np.matlib.repmat(scan_avg,1,4)
                self.scan = scan_avg_map.reshape(6400)
                # MaxAbsScaler:
                s_min = 0
                s_max = self.LASER_CLIP
                self.scan = 2 * (self.scan - s_min) / (s_max - s_min) + (-1)

                # goal:
                # MaxAbsScaler:
                g_min = -1
                g_max = 1
                goal_orignal = np.array(self.goal, dtype=np.float32)
                self.goal = 2 * (goal_orignal - g_min) / (g_max - g_min) + (-1)

                # observation:
                self.observation = np.concatenate((ped_pos, self.scan, self.goal), axis=None) 

                # drl-vo infrence: calculate the goal velocity of the robot and send the command
                action, _states = self.model.predict(self.observation, deterministic=True)

                # velocities:
                vx_min = 0
                if(min_scan_dist >= 2.2): # free space margin
                    vx_max = 2
                else:
                    vx_max = 0.5 
                vz_min = -0.7
                vz_max = 0.7
                # MaxAbsScaler inverse:
                cmd_vel.linear.x = (action[0] + 1) * (vx_max - vx_min) / 2 + vx_min
                cmd_vel.angular.z = (action[1] + 1) * (vz_max - vz_min) / 2 + vz_min
        
        # publish the cmd_vel:
        if not np.isnan(cmd_vel.linear.x) and not np.isnan(cmd_vel.angular.z): # ensure data is valid
            self.cmd_vel_pub.publish(cmd_vel)
    
    def _clear_costmap(self):
        self.move_base.clear_costmap()
        rospy.sleep(0.1)
        self.move_base.clear_costmap()
        rospy.sleep(0.1)
        self.move_base.clear_costmap()
    #
    # end of function


# begin gracefully
#

if __name__ == '__main__':
    rospy.init_node('drl_vo_inference')
    drl_infe = DrlInference()
    rospy.spin()

# end of file
