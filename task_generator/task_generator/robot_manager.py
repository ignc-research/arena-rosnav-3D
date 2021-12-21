#!/usr/bin/env python


import rospy, math
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped
from gazebo_msgs.srv import SetModelState, SpawnModelRequest, SpawnModel
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Path
from .utils import generate_freespace_indices, get_random_pos_on_map

ROBOT_RADIUS = rospy.get_param("radius")


class RobotManager:
    """
    A manager class using gazebo provided services to spawn, move and delete Robot. Currently only one robot
    is managed
    """

    def __init__(self, ns, map_):
        # type (str, OccupancyGrid, str, int) -> None
        """[summary]
        Args:
            ns(namespace): if ns == '', we will use global namespace
            map_ (OccupancyGrid): the map info
        """
        self.ns = ns
        self.ns_prefix = "/" if ns == "" else "/" + ns + "/"
        self.ROBOT_NAME = "turtlebot3"
        self.ROBOT_DESCRIPTION = rospy.get_param("robot_description")
        self.update_map(map_)

        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")

        self._srv_spawn_model = rospy.ServiceProxy(
            "/gazebo/spawn_urdf_model", SpawnModel
        )
        self._goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=1, latch=True)
        self.pub_mvb_goal = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )
        self.planer = rospy.get_param("local_planner")
        self.spawn_robot()

    def update_map(self, new_map):
        # type (OccupancyGrid) -> None
        self.map = new_map
        self._free_space_indices = generate_freespace_indices(self.map)

    def spawn_robot(self):
        request = SpawnModelRequest()
        request.model_name = self.ROBOT_NAME
        request.model_xml = self.ROBOT_DESCRIPTION
        request.robot_namespace = self.ns_prefix + self.ns
        request.initial_pose = Pose()
        request.initial_pose.position.z = 0.2
        request.reference_frame = "world"
        self._srv_spawn_model(request)

    def move_robot(self, pose):
        # type: (Pose) -> None
        """move the robot to a given position
        Args:
            pose (Pose): target postion
        """
        start_pos = ModelState()
        start_pos.model_name = "turtlebot3"
        start_pos.pose = pose
        start_pos.pose.position.z = 0.2
        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            set_state(start_pos)

        except rospy.ServiceException:
            print("Move Robot to position failed")

        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

        start_pos = PoseWithCovarianceStamped()
        start_pos.header.frame_id = "map"
        start_pos.pose.pose = pose
        pub.publish(start_pos)

    def publish_goal(self, pose):
        # type: (Pose) -> None
        """
        Publishing goal (x, y, theta)
        :param x x-position of the goal
        :param y y-position of the goal
        :param theta theta-position of the goal
        """
        self._global_path = Path()
        self._old_global_path_timestamp = self._global_path.header.stamp
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose = pose
        self._goal_pub.publish(goal)

        # these planer need the nav-goal pulished to the /move_base_simple/goal topic
        if self.planer in ["teb", "dwa", "mpc"]:
            # Make sure move_base is ready to take goals
            rospy.wait_for_service("/move_base/make_plan")
            self.pub_mvb_goal.publish(goal)

    def set_start_pos_random(self):
        start_pos = Pose()
        start_pos = get_random_pos_on_map(
            self._free_space_indices, self.map, ROBOT_RADIUS
        )
        self.move_robot(start_pos)
        return start_pos

    def set_start_pos_goal_pos(
        self, start_pos=None, goal_pos=None, min_dist=1, forbidden_zones=None
    ):
        # type: (Union[Pose, None], Union[Pose, None], int, list) -> float
        """set up start position and the goal postion. Path validation checking will be conducted. If it failed, an
        exception will be raised.
        Args:
            start_pos (Union[Pose2D,None], optional): start position. if None, it will be set randomly. Defaults to None.
            goal_pos (Union[Pose2D,None], optional): [description]. if None, it will be set randomly .Defaults to None.
            min_dist (float): minimum distance between start_pos and goal_pos
        Exception:
            Exception("can not generate a path with the given start position and the goal position of the robot")
        """

        def dist(x1, y1, x2, y2):
            return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

        if start_pos is None or goal_pos is None:
            # if any of them need to be random generated, we set a higher threshold,otherwise only try once
            max_try_times = 20
        else:
            max_try_times = 1
        if forbidden_zones == None:
            forbidden_zones = None  # change later
        i_try = 0
        start_pos_ = None
        goal_pos_ = None

        while i_try < max_try_times:

            if start_pos is None:
                start_pos_ = get_random_pos_on_map(
                    self._free_space_indices,
                    self.map,
                    ROBOT_RADIUS * 2,
                    forbidden_zones,
                )
            else:
                start_pos_ = start_pos

            if goal_pos is None:
                goal_pos_ = get_random_pos_on_map(
                    self._free_space_indices,
                    self.map,
                    ROBOT_RADIUS * 2,
                    forbidden_zones,
                )
            else:
                goal_pos_ = goal_pos

            if (
                dist(
                    start_pos_.position.x,
                    start_pos_.position.y,
                    goal_pos_.position.x,
                    goal_pos_.position.y,
                )
                < min_dist
            ):
                i_try += 1
                continue
            # move the robot to the start pos
            self.move_robot(start_pos_)
            try:
                # publish the goal, if the global plath planner can't generate a path, a, exception will be raised.
                self.publish_goal(goal_pos_)
                break
            except rospy.ServiceException:
                i_try += 1
        if i_try == max_try_times:
            # TODO Define specific type of Exception
            raise rospy.ServiceException(
                "can not generate a path with the given start position and the goal position of the robot"
            )
        else:
            return start_pos_, goal_pos_
