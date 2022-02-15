import json
from collections import deque
from copy import deepcopy

import nav_msgs
import numpy as np
import rospy
# services
import visualization_msgs.msg
from geometry_msgs.msg import Pose2D, PoseStamped, Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
# observation msgs
from sensor_msgs.msg import LaserScan
# for transformations
from tf.transformations import *
from visualization_msgs.msg import Marker


class ObservationCollectorAllInOne:
    def __init__(self, required_obs: dict, all_in_one_config: str):

        # hardcoded laser settings of AIO planner
        self._laser_num_beams = rospy.get_param("laser_beams")
        self.max_range_aio_planner = rospy.get_param("laser_range")

        self._reduced_num_laser_beams, self._laser_stack_size, self._add_robot_velocity, self._use_dynamic_scan = \
            self._extract_obs_space_info(all_in_one_config)
        self._full_laser_range = self._reduced_num_laser_beams < 0
        self._laser_vec_size = self._laser_num_beams if self._full_laser_range else self._reduced_num_laser_beams
        if self._use_dynamic_scan:
            self._laser_vec_size = 2 * self._laser_vec_size

        self._stacked_obs_space = self._laser_stack_size > 1

        # print observation space information
        rospy.loginfo("Observation space configuration:")
        rospy.loginfo("Use laser info of size ({0},{1})".format(self._laser_stack_size, self._laser_vec_size))
        rospy.loginfo("Use robot velocity: {}".format(self._add_robot_velocity))

        if self._stacked_obs_space:
            self._obs_stacked = np.zeros((self._laser_stack_size, self._laser_vec_size))
            self._obs_stacked_empty = True

        self._required_obs = required_obs

        self._clock = Clock()
        self._scan = LaserScan()
        self._scan_static = LaserScan()
        self._robot_pose = Pose2D()
        self._robot_vel = Twist()
        self._goal = Pose2D()
        self._goal3D = Pose()
        self._subgoal = Pose2D()
        self._subgoal_reward = Pose2D()
        self._old_subgoal = Pose2D()
        self._globalplan = np.array([])
        self._globalplan_raw = nav_msgs.msg.Path()
        self._twist = Twist()

        self._drl_subgoal_horizon = 2.7

        self._AIO_subgoal_horizon = 0.7

        # synchronization parameters
        self._first_sync_obs = True  # whether to return first sync'd obs or most recent
        self.max_deque_size = 10
        self._sync_slop = 0.05

        self._laser_deque = deque()
        self._laser_static_deque = deque()
        self._rs_deque = deque()

        self._dyn_scan_msg = None

        # subscriptions
        self._scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan, tcp_nodelay=True)
        if self._use_dynamic_scan:
            self._scan_static_sub = rospy.Subscriber('/static_scan', LaserScan, self.callback_scan_static,
                                                     tcp_nodelay=True)
        self._robot_state_sub = rospy.Subscriber('/odom', Odometry, self.callback_robot_state, tcp_nodelay=True)
        self._goal_sub = rospy.Subscriber('/goal', PoseStamped, self.callback_goal)

        self._globalplan_sub = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', nav_msgs.msg.Path, self.callback_global_plan,
                                                tcp_nodelay=True)

        # visualization publishers
        self._dynamic_scan_pub = rospy.Publisher('all_in_one_planner/scan_dynamic', LaserScan, queue_size=1)
        self._subgoal_visualizer = rospy.Publisher('all_in_one_planner/vis_subgoal', visualization_msgs.msg.Marker,
                                                   queue_size=1)

        if 'laser_3' in self._required_obs and self._required_obs['laser_3']:
            self._last_three_laser_scans = np.zeros((self._laser_num_beams, 3))
            self._needs_last_three_laser = True
        else:
            self._needs_last_three_laser = False

    def get_observations(self):
        # get synced observations from queue
        laser_scan, laser_scan_static, robot_pose, twist = self.get_sync_obs()

        # check if they are empty (start of simulation)
        if laser_scan is not None and robot_pose is not None and twist is not None:
            self._scan = laser_scan
            self._robot_pose = robot_pose
            self._twist = twist
        if self._use_dynamic_scan and laser_scan_static is not None:
            self._scan_static = laser_scan_static

        # If not empty, convert them to numpy
        if len(self._scan.ranges) > 0:
            scan_full = self._scan.ranges.astype(np.float32)
        else:
            scan_full = np.zeros(self._laser_num_beams, dtype=float)
        if self._use_dynamic_scan and len(self._scan_static.ranges) > 0:
            scan_static = self._scan_static.ranges.astype(np.float32)
            dynamic_scan = self.get_dynamic_scan(scan_static, scan_full)
        else:
            scan_static = np.zeros(self._laser_num_beams, dtype=float)
            dynamic_scan = np.zeros(self._laser_num_beams, dtype=float)

        # extract subgoal from new global plan
        self._subgoal = self._extract_subgoal(self._globalplan, self._drl_subgoal_horizon)
        self._subgoal_reward = self._extract_subgoal(self._globalplan, self._AIO_subgoal_horizon)
        # visualize subgoal
        self._visualize_subgoal(self._subgoal)

        # Convert goals to robot frame
        rho_local, theta_local = ObservationCollectorAllInOne._get_goal_pose_in_robot_frame(
            self._subgoal, self._robot_pose)

        rho_global, theta_global = ObservationCollectorAllInOne._get_goal_pose_in_robot_frame(
            self._goal, self._robot_pose)

        local_goal_x, local_goal_y = ObservationCollectorAllInOne._get_local_goal_in_robot_frame_xy(
            self._subgoal, self._robot_pose)

        subgoal_reward_robot_frame_rho, subgoal_reward_robot_frame_theta = ObservationCollectorAllInOne._get_goal_pose_in_robot_frame(
            self._subgoal_reward, self._robot_pose)

        # If specified reduce number of laser beams
        if self._full_laser_range:
            if self._use_dynamic_scan:
                laser_vec = np.hstack([scan_static, dynamic_scan])
            else:
                laser_vec = scan_full
        else:
            if self._use_dynamic_scan:
                dyn_vec = self._calc_reduced_laser_vec(dynamic_scan)
                static_vec = self._calc_reduced_laser_vec(scan_static)
                laser_vec = np.hstack([static_vec, dyn_vec])
            else:
                laser_vec = self._calc_reduced_laser_vec(scan_full)

        # If specified stack last x observations
        if self._stacked_obs_space:
            if self._obs_stacked_empty:
                # First iteration, only use first scan
                self._obs_stacked[:, :] = self._laser_stack_size * [laser_vec]
                self._obs_stacked_empty = False
            else:
                # Replace first entry and shift rest
                self._obs_stacked[1:, :] = self._obs_stacked[:-1, :]
                self._obs_stacked[0, :] = laser_vec
            laser_vec = self._obs_stacked.flatten()

        merged_obs = np.float32(
            np.hstack([laser_vec, np.array([rho_global, theta_global]), np.array([rho_local, theta_local])]))

        # If specified add robot twist to observation
        if self._add_robot_velocity:
            merged_obs = np.hstack([merged_obs, [self._twist.linear.x, self._twist.angular.z]])

        obs_dict = {'laser_scan': scan_full,
                    'goal_map_frame': self._subgoal,
                    'goal_reward_robot_frame': [subgoal_reward_robot_frame_rho, subgoal_reward_robot_frame_theta],
                    'goal_in_robot_frame': [rho_local, theta_local],
                    'goal_in_robot_frame_xy': [local_goal_x, local_goal_y],
                    'global_plan': self._globalplan,
                    'global_plan_raw': self._globalplan_raw,
                    'robot_pose': self._robot_pose,
                    'robot_twist': self._twist,
                    'global_goal': np.array([self._goal.x, self._goal.y]),
                    'global_goal_robot_frame': np.array([rho_global, theta_global])
                    }
        if self._use_dynamic_scan:
            obs_dict['scan_dynamic'] = dynamic_scan

        # if necessary add last 3 laser scans to obs dict
        if self._needs_last_three_laser:
            # self._last_three_laser_scans[:, 1:2] = self._last_three_laser_scans[:, 0:1]
            # self._last_three_laser_scans[:, 0] = scan
            self._last_three_laser_scans[:, 0] = scan_full
            self._last_three_laser_scans[:, 1] = scan_full
            self._last_three_laser_scans[:, 2] = scan_full
            obs_dict['laser_3'] = self._last_three_laser_scans

        self._laser_deque.clear()
        self._laser_static_deque.clear()
        self._rs_deque.clear()

        return merged_obs, obs_dict

    def reset(self):
        if self._stacked_obs_space:
            self._obs_stacked = np.zeros((self._laser_stack_size, self._laser_vec_size))
            self._obs_stacked_empty = True

        if self._needs_last_three_laser:
            if len(self._scan.ranges) > 0:
                scan = self._scan.ranges.astype(np.float32)
            else:
                scan = np.zeros(self._laser_num_beams, dtype=float)
            self._last_three_laser_scans = np.array(
                [scan, scan, scan]).transpose()

    def get_dynamic_scan(self, static_scan: np.ndarray, scan: np.ndarray):
        diff_scan = np.abs(scan - static_scan)
        dynamic_scan = np.where(diff_scan < 0.5, self.max_range_aio_planner, scan)

        dynamic_scan = self.filter_outliers(dynamic_scan, self.max_range_aio_planner)

        dynamic_scan = np.where(dynamic_scan > 4, self.max_range_aio_planner, dynamic_scan)

        # publish dynamic scan for visualization
        self._dyn_scan_msg.ranges = dynamic_scan
        self._dyn_scan_msg.header.stamp = rospy.get_rostime()
        self._dynamic_scan_pub.publish(self._dyn_scan_msg)
        return dynamic_scan

    def get_sync_obs(self):
        laser_scan = None
        static_scan = None
        robot_pose = None
        twist = None
        laser_stamp = None

        # print(f"laser deque: {len(self._laser_deque)}, robot state deque: {len(self._rs_deque)}")
        while len(self._rs_deque) > 0 and len(self._laser_deque) > 0:
            laser_scan_msg = self._laser_deque.popleft()
            robot_pose_msg = self._rs_deque.popleft()

            laser_stamp = laser_scan_msg.header.stamp.to_sec()
            robot_stamp = robot_pose_msg.header.stamp.to_sec()

            while not abs(laser_stamp - robot_stamp) <= self._sync_slop:
                if laser_stamp > robot_stamp:
                    if len(self._rs_deque) == 0:
                        return laser_scan, robot_pose, twist
                    robot_pose_msg = self._rs_deque.popleft()
                    robot_stamp = robot_pose_msg.header.stamp.to_sec()
                else:
                    if len(self._laser_deque) == 0:
                        return laser_scan, robot_pose, twist
                    laser_scan_msg = self._laser_deque.popleft()
                    laser_stamp = laser_scan_msg.header.stamp.to_sec()

            laser_scan = self.process_scan_msg(laser_scan_msg)
            robot_pose, twist = self.process_robot_state_msg(robot_pose_msg)

            if self._first_sync_obs:
                break

        if self._use_dynamic_scan:
            # Extract static scan
            static_laser_stamp = 0
            static_laser_scan_msg = None
            if laser_stamp is not None:
                while not abs(laser_stamp > static_laser_stamp) <= self._sync_slop and len(
                        self._laser_static_deque) > 0:
                    static_laser_scan_msg = self._laser_static_deque.popleft()
                    static_laser_stamp = static_laser_scan_msg.header.stamp.to_sec()

                if static_laser_scan_msg is not None:
                    static_scan = self.process_scan_msg(static_laser_scan_msg)
                    self._dyn_scan_msg = static_laser_scan_msg

            # print(f"Laser_stamp: {laser_stamp}, Static_stamp: {static_laser_stamp}")

        return laser_scan, static_scan, robot_pose, twist

    def callback_clock(self, msg_Clock):
        self._clock = msg_Clock.clock.to_sec()
        return

    def callback_subgoal(self, msg_Subgoal):
        self._subgoal = self.process_subgoal_msg(msg_Subgoal)
        return

    def callback_goal(self, msg_Goal):
        self._goal = self.pose3D_to_pose2D(msg_Goal.pose)
        self._goal3D = msg_Goal.pose
        return

    def callback_global_plan(self, msg_global_plan: nav_msgs.msg.Path):
        self._globalplan = ObservationCollectorAllInOne.process_global_plan_msg(msg_global_plan)
        self._globalplan_raw = msg_global_plan
        return

    def callback_scan(self, msg_laserscan: LaserScan):
        # save message
        if len(self._laser_deque) == self.max_deque_size:
            self._laser_deque.popleft()
        self._laser_deque.append(msg_laserscan)

    def callback_scan_static(self, msg_laserscan: LaserScan):
        # save message
        if len(self._laser_static_deque) == self.max_deque_size:
            self._laser_static_deque.popleft()
        self._laser_static_deque.append(msg_laserscan)

    def callback_robot_state(self, msg_robotstate):
        if len(self._rs_deque) == self.max_deque_size:
            self._rs_deque.popleft()
        self._rs_deque.append(msg_robotstate)

    def callback_observation_received(self, msg_LaserScan, msg_RobotStateStamped):
        # process sensor msg
        self._scan = self.process_scan_msg(msg_LaserScan)
        self._robot_pose, self._robot_vel = self.process_robot_state_msg(msg_RobotStateStamped)
        self.obs_received = True
        return

    def process_scan_msg(self, msg_LaserScan: LaserScan):
        # remove_nans_from_scan
        self._scan_stamp = msg_LaserScan.header.stamp.to_sec()
        scan = np.array(msg_LaserScan.ranges)
        scan[np.isnan(scan)] = self.max_range_aio_planner
        scan[scan > self.max_range_aio_planner] = self.max_range_aio_planner
        msg_LaserScan.ranges = scan
        return msg_LaserScan

    def process_robot_state_msg(self, msg_Odometry):
        pose3d = msg_Odometry.pose.pose
        twist = msg_Odometry.twist.twist
        twist.linear.x = np.linalg.norm([twist.linear.x, twist.linear.y])
        twist.linear.y = 0
        return self.pose3D_to_pose2D(pose3d), twist

    def process_pose_msg(self, msg_PoseWithCovarianceStamped):
        # remove Covariance
        pose_with_cov = msg_PoseWithCovarianceStamped.pose
        pose = pose_with_cov.pose
        return self.pose3D_to_pose2D(pose)

    def process_subgoal_msg(self, msg_Subgoal):
        pose2d = self.pose3D_to_pose2D(msg_Subgoal.pose)
        return pose2d

    def _calc_reduced_laser_vec(self, laser_scan: np.array):
        return list(map(np.min, np.array_split(laser_scan, self._reduced_num_laser_beams)))

    def _visualize_subgoal(self, subgoal: Pose2D):
        subgoal_msg = Marker()
        subgoal_msg.header.frame_id = "map"
        subgoal_msg.id = 105
        subgoal_msg.action = Marker.ADD
        subgoal_msg.type = Marker.SPHERE
        subgoal_msg.color.r = 1
        subgoal_msg.color.g = 1
        subgoal_msg.color.b = 0
        subgoal_msg.color.a = 0.7
        subgoal_msg.scale.x = 0.7
        subgoal_msg.scale.y = 0.7
        subgoal_msg.scale.z = 0.01

        subgoal_msg.pose.position.x = subgoal.x
        subgoal_msg.pose.position.y = subgoal.y

        self._subgoal_visualizer.publish(subgoal_msg)

    def _extract_subgoal(self, global_plan, lookahead_dist: float) -> Pose2D:
        # extract subgoal by getting point on global path with @_planning_horizon distance
        if global_plan.size == 0:
            return self._goal

        start_xy = global_plan[0]
        end_xy = global_plan[-1]

        if global_plan.size <= 2:
            return Pose2D(end_xy[0], end_xy[1], 0)

        if np.linalg.norm(start_xy - end_xy) < lookahead_dist:
            return Pose2D(end_xy[0], end_xy[1], 0)

        i = 1
        next_pose_xy = global_plan[i]
        while np.linalg.norm(start_xy - next_pose_xy) < lookahead_dist and i < global_plan.size - 1:
            i += 1
            next_pose_xy = global_plan[i]

        return Pose2D(next_pose_xy[0], next_pose_xy[1], 0)

    @staticmethod
    def _extract_obs_space_info(all_in_one_config_path: str):
        with open(all_in_one_config_path, 'r') as config_json:
            all_in_one_config = json.load(config_json)

        assert all_in_one_config is not None, "All in one parameter file cannot be found!"

        if 'observation_space' not in all_in_one_config:
            rospy.logwarn("No specification of the observation space found. Use default values!")
            return -1, 1, False, True
        else:
            obs_info = all_in_one_config['observation_space']
            assert 'laser_range' in obs_info, "Please specify the laser_range parameter in the observation " \
                                              "space section of the aio config file!"
            if obs_info['laser_range'] == 'full':
                reduced_laser_size = -1
            else:
                assert 'size_laser_vector' in obs_info, "Please specify the size_laser_vector parameter in the " \
                                                        "aoi config file or use laser_range=full setting otherwise!"
                reduced_laser_size = obs_info['size_laser_vector']
            assert 'laser_stack_size' in obs_info, "Please specify the laser_stack_size parameter in the aoi " \
                                                   "config file!"
            laser_stack_size = obs_info['laser_stack_size']

            if 'add_robot_velocity' in obs_info and obs_info['add_robot_velocity']:
                add_robot_velocity = True
            else:
                add_robot_velocity = False

            if 'use_dynamic_scan' not in obs_info or ('use_dynamic_scan' in obs_info and obs_info['use_dynamic_scan']):
                use_dynamic_scan = True
            else:
                use_dynamic_scan = False

            return reduced_laser_size, laser_stack_size, add_robot_velocity, use_dynamic_scan

    @staticmethod
    def process_global_plan_msg(globalplan):
        global_plan_2d = list(map(
            lambda p: ObservationCollectorAllInOne.pose3D_to_pose2D(p.pose), globalplan.poses))
        global_plan_np = np.array(list(map(lambda p2d: [p2d.x, p2d.y], global_plan_2d)))
        return global_plan_np

    @staticmethod
    def pose3D_to_pose2D(pose3d):
        pose2d = Pose2D()
        pose2d.x = pose3d.position.x
        pose2d.y = pose3d.position.y
        quaternion = (pose3d.orientation.x, pose3d.orientation.y,
                      pose3d.orientation.z, pose3d.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        pose2d.theta = yaw
        return pose2d

    @staticmethod
    def _get_local_goal_in_robot_frame_xy(goal_pos: Pose2D, robot_pos: Pose2D):
        [x, y, theta] = [robot_pos.x, robot_pos.y, robot_pos.theta]  # self position based on map
        [goal_x, goal_y] = [goal_pos.x, goal_pos.y]  # sub goal based on map
        local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
        local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
        return [local_x, local_y]  # return subgoal position based on robot

    @staticmethod
    def _get_goal_pose_in_robot_frame(goal_pos: Pose2D, robot_pos: Pose2D):
        y_relative = goal_pos.y - robot_pos.y
        x_relative = goal_pos.x - robot_pos.x
        rho = (x_relative ** 2 + y_relative ** 2) ** 0.5
        theta = (np.arctan2(y_relative, x_relative) -
                 robot_pos.theta + 4 * np.pi) % (2 * np.pi) - np.pi
        return rho, theta

    @staticmethod
    def filter_outliers(scan: np.ndarray, max_range: float):
        # remove scan outlier areas of size 1 and 2

        filtered_scan = deepcopy(scan)
        diff_tolerance = 0.2

        def left_edge(array):
            return abs(array[0] - array[1]) > diff_tolerance

        def right_edge(array):
            return abs(array[2] - array[1]) > diff_tolerance

        def left_long_edge(array):
            return abs(array[0] - array[2]) > diff_tolerance

        def right_long_edge(array):
            return abs(array[2] - array[4]) > diff_tolerance

        # easy way to remove outliers
        for i in range(2, scan.size - 2):
            short_slice = scan[i - 1:i + 2]

            is_left_edge = left_edge(short_slice)
            is_right_edge = right_edge(short_slice)

            is_outlier = False

            if is_left_edge or is_right_edge:
                long_slice = scan[i - 2:i + 3]
                if is_left_edge and is_right_edge:
                    is_outlier = True
                elif is_left_edge and right_long_edge(long_slice):
                    is_outlier = True
                elif is_right_edge and left_long_edge(long_slice):
                    is_outlier = True

                if is_outlier:
                    filtered_scan[i] = max_range

        return filtered_scan
