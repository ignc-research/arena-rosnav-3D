#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  1 21:24:03 2022

@author: bezzo
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import math as m
import numpy as np


class messageClass:
    def __init__(self):
        self.goalx = None
        self.goaly = None
        self.velx = 0.0
        self.velz = 0.0
        self.target_velocity = 2.0
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0
        self.scandata = None
        self.ranges = []
        self.angle_inc = None
        self.scanCoords = []
        self.angles = []
        self.gapGoal = None
        self.buffer = 0.5
        self.footpr = 0.8
        self.minind = 0
        self.maxind = 719
        self.angle_min = None


def distance(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


def gapGoalCallback(data):
    message.gapGoal = data.data[:2]


def yawFromQuaternion(orientation):
    return m.atan2(
        (2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)),
        (1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)),
    )


def scanCallback(data):
    message.scandata = np.asarray(data.ranges[message.minind : message.maxind])
    message.angle_inc = data.angle_increment
    if not len(message.angles):
        message.angles = np.array(
            [
                data.angle_min + i * data.angle_increment
                for i in range(message.minind, message.maxind)
            ]
        )
        message.angle_min = data.angle_min


def odomCallback(data):
    message.velx = data.twist.twist.linear.x
    message.velz = data.twist.twist.angular.z
    message.posx = data.pose.pose.position.x
    message.posy = data.pose.pose.position.y
    message.theta = yawFromQuaternion(data.pose.pose.orientation)


def wrapToPi(angle):
    if angle > m.pi:
        angle -= 2 * m.pi
    elif angle < -m.pi:
        angle += 2 * m.pi
    return angle


def ray_check(goal):
    marker_pub = rospy.Publisher(
        "/visualization_marker_array", MarkerArray, queue_size=0
    )
    marker_arr = MarkerArray()
    # scanDataCoords()
    goal_dist = distance(goal, (message.posx, message.posy))
    comp_dist = 1
    r_dist = goal_dist
    theta_d = m.atan2(goal[1] - message.posy, goal[0] - message.posx)
    ang_err = wrapToPi(
        theta_d - message.theta
    )  # m.atan2(m.sin(theta_d-message.theta),m.cos(theta_d-message.theta))
    kp = 0.55
    scan_ind = np.argmin(abs(message.angles - ang_err))
    replan = True
    r_goal = goal
    count = 0
    while replan == True:
        if goal_dist > 0.8:
            check_obs = []
            check_dist = []
            ab_dist = min(r_dist, comp_dist)  # goal_dist
            val = (ab_dist**2 + ab_dist**2 - message.footpr**2) / (
                2 * ab_dist * goal_dist
            )
            val = min(max(val, -1), 1)
            check_rays = m.ceil(np.arccos(val) / message.angle_inc * 0.5)
            # print(check_rays)
            # print(max(scan_ind-check_rays,message.minind),min(scan_ind+check_rays,message.maxind))
            for idx in range(
                int(max(scan_ind - check_rays, message.minind)),
                int(min(scan_ind + check_rays, message.maxind)),
            ):
                if message.scandata[idx] < ab_dist:
                    if idx < scan_ind:
                        check_obs.append(1)
                        check_dist.append(message.scandata[idx])
                    else:
                        check_obs.append(-1)
                        check_dist.append(message.scandata[idx])
                else:
                    check_obs.append(0)
                    check_dist.append(500)
            min_idx = np.min(message.scandata)
            if min_idx < comp_dist:
                check_obs.append(1 if min_idx < scan_ind else -1)
                # check_dist.append(message.scandata[min_idx])

            if np.sum(check_obs) != 0:
                replan = True
                scan_ind += np.sum(check_obs)
                if scan_ind < message.minind or scan_ind > message.maxind:

                    replan = False
                else:
                    r_dist = min(check_dist)
                    r_goal = [
                        message.posx
                        + r_dist * m.cos(message.angles[scan_ind] + message.theta),
                        message.posy
                        + r_dist * m.sin(message.angles[scan_ind] + message.theta),
                    ]
                    theta_d = m.atan2(
                        r_goal[1] - message.posy, r_goal[0] - message.posx
                    )
                    ang_err = m.atan2(
                        m.sin(theta_d - message.theta), m.cos(theta_d - message.theta)
                    )
            else:
                replan = False
        else:
            replan = False

        marker_arr.markers.append(generate_wpt_marker(r_goal))
        marker_pub.publish(marker_arr)

        count += 1
        if count >= 10:
            # print("replanning found nothing")
            replan = False
            # r_goal = goal
            # kp = 0.25
        # if r_dist<0.6:
        # kp = 0.25

    return theta_d, r_goal, kp


def generate_wpt_marker(wpt):

    msg = Marker()
    if len(wpt) == 0:
        return msg

    msg.header.frame_id = "odom"
    msg.header.stamp = rospy.Time.now()
    msg.ns = "local_goal"
    msg.id = 281
    msg.action = Marker.ADD
    msg.pose.position.x = wpt[0]
    msg.pose.position.y = wpt[1]
    msg.pose.orientation.w = 1
    msg.lifetime = rospy.Duration()
    msg.type = Marker.SPHERE
    msg.scale.x = msg.scale.y = msg.scale.z = 0.3
    msg.color.r, msg.color.g, msg.color.b, msg.color.a = (0, 1, 1, 1)

    return msg


def potential_field(des_ang):
    scanDataCoords()
    obs_forces_x = []
    obs_forces_y = []
    for idx in range(len(message.scandata)):
        if message.scandata[idx] < message.buffer:
            obs_forces_x.append(
                (message.scanCoords[idx][0] - message.posx)
                / (message.scandata[idx]) ** 2
            )
            obs_forces_y.append(
                (message.scanCoords[idx][1] - message.posy)
                / (message.scandata[idx]) ** 2
            )
    obs_force_x = 0 if len(obs_forces_x) < 1 else sum(obs_forces_x) / len(obs_forces_x)
    obs_force_y = 0 if len(obs_forces_y) < 1 else sum(obs_forces_y) / len(obs_forces_y)

    force_x = -0.1 * obs_force_x
    force_y = -0.1 * obs_force_y

    if force_x == 0 and force_y == 0:
        theta_d = des_ang
    else:
        force_ang = m.atan2(force_y, force_x)
        theta_d = m.atan2(m.sin(des_ang + force_ang), m.cos(des_ang + force_ang))

    return theta_d


def scanDataCoords():
    coords = []
    for scan in range(len(message.scandata)):
        angle = wrapToPi(message.angles[scan] + message.theta)
        xCoord = message.posx + message.scandata[scan] * m.cos(angle)
        yCoord = message.posy + message.scandata[scan] * m.sin(angle)
        coords.append((xCoord, yCoord))
    message.scanCoords = coords


def my_shutdown_hook():
    rospy.loginfo("It's shutdown time!")


def main():
    global message
    message = messageClass()
    gaps = rospy.Subscriber("/gapGoal", Float32MultiArray, gapGoalCallback)
    odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback)
    scan_sub = rospy.Subscriber("/scan", LaserScan, scanCallback)
    vel_Pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rospy.init_node("old_mcdonalds_local", anonymous=True)
    rate = rospy.Rate(100)  # 10hz

    maxSpeed = 1
    minSpeed = 0.1
    maxTurn = m.pi / 2
    isDone = False
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    message.goalx = message.posx
    message.goaly = message.posy + 10
    kp = 0.55
    kt = 1.1
    print("I started")
    while not rospy.is_shutdown():
        if message.angle_inc == None or message.gapGoal == None:
            continue

        localGoal = tuple(message.gapGoal)
        des_ang, r_goal, kp = ray_check(localGoal)
        theta_d = des_ang
        ang_err = m.atan2(
            m.sin(theta_d - message.theta), m.cos(theta_d - message.theta)
        )
        ang_v = np.sign(ang_err) * min(maxTurn, kt * abs(ang_err))
        twist.angular.z = ang_v
        # lin_prop = (1-minSpeed)/(1+m.exp(8*(ang_err-m.pi/8)))
        # twist.linear.x = kp*maxSpeed*lin_prop+minSpeed
        twist.linear.x = kp * maxSpeed * (1 - abs(ang_v) / maxTurn)

        distToGoal = distance(
            (message.goalx, message.goaly), (message.posx, message.posy)
        )

        planToGoal = distance((message.goalx, message.goaly), (r_goal[0], r_goal[1]))

        if planToGoal <= 0.5:
            kp = 0.75

        if distToGoal < 0.5:
            twist.linear.x = 0
            twist.angular.z = 0
            print("I stahped")
            isDone = True

        vel_Pub.publish(twist)

        if isDone:
            rospy.signal_shutdown("Goal Reached")

        rate.sleep()


if __name__ == "__main__":
    #    try:
    main()
#    except:
#        rospy.on_shutdown(my_shutdown_hook)
