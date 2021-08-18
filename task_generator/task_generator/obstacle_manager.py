#!/usr/bin/env python


#from .tasks import PedsimManager
from .utils import generate_freespace_indices, get_random_pos_on_map
from gazebo_msgs.srv import GetWorldProperties
import rospy
from pedsim_msgs.msg import AgentStates

class ObstaclesManager:


    def __init__(self, ns, map_):
        # tpye (str, OccupancyGrid)
        """
        Args:
            map_ (OccupancyGrid):
            plugin_name: The name of the plugin which is used to control the movement of the obstacles, Currently we use "RandomMove" for training and Tween2 for evaluation.
                The Plugin Tween2 can move the the obstacle along a trajectory which can be assigned by multiple waypoints with a constant velocity.Defaults to "RandomMove".
        """
        self.ns = ns
        self.ns_prefix = "" if ns == '' else "/"+ns+"/"

        # a list of publisher to move the obstacle to the start pos.
        self._move_all_obstacles_start_pos_pubs = []

        self.update_map(map_)
        self.obstacle_name_list = []
        self._obstacle_name_prefix = 'obstacle'
        # remove all existing obstacles generated before create an instance of this class
        #self.remove_obstacles()

        self.OBSTACLE_RADIUS = 0,15

    def update_map(self, new_map):
        # type (OccupancyGrid)-> None
        self.map = new_map
        # a tuple stores the indices of the non-occupied spaces. format ((y,....),(x,...)
        self._free_space_indices = generate_freespace_indices(self.map)

    def remove_obstacles(self):
        removeAllPeds()
        #ToDo remove them in gazebo as well

    def register_random_dynamic_obstacles(self, num_obstacles, forbidden_zones = None): # [(start_pos.x, start_pos.y, self.robot_manager.ROBOT_RADIUS), robot goal...]
        # type: (int, list) -> None
        
        """register dynamic obstacles (humans) with random start positions
        Args:
            num_obstacles (int): number of the obstacles.

        """
        for obstacle in range(num_obstacles):
            start_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)

            def dist(x1, y1, x2, y2):
                return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

            max_try_times = 20
            i_try = 0
            start_pos_ = None
            goal_pos_ = None
        
            while i_try < max_try_times:
                start_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)
                goal_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)

                if dist(start_pos_.position.x, start_pos_.position.y, goal_pos_.position.x, goal_pos_.position.y) < min_dist: 
                    i_try += 1
                    continue
                try:
                    setup_spawn_ped(obstacle)
                    # spawn the obstacle with this coordinates and waypoint
                    forbidden_zones.append((start_pos.pose.x, start_pos.pose.y, self.OBSTACLE_RADIUS))
                    break
                except rospy.ServiceException:
                    i_try += 1
        if i_try == max_try_times:
            # TODO Define specific type of Exception
            raise rospy.ServiceException(
                "can not generate a path with the given start position and the goal position of the robot")
        else:
            return start_pos_, goal_pos_

    def reset_pos_obstacles_random(self, forbidden_zones = None):
            # type: (list) -> None
            elements = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldPropertis)
            for ped in range(elements - 3): # Todo how to get the current Agents?
                start_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)
                goal_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)
            #idee: 1. find all obstacles; 2.  for every obstacle call the move ped service