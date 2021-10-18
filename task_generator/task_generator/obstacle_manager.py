#!/usr/bin/env python


import rospy, math, rospkg
from random import randint, uniform, choice
from .utils import generate_freespace_indices, get_random_pos_on_map
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from .ped_manager.ArenaScenario import *
from .pedsim_manager import PedsimManager


STANDART_ORIENTATION = quaternion_from_euler(0.0,0.0,0.0)


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
        # self.remove_obstacles()

        self.OBSTACLE_RADIUS = 0.15

    def update_map(self, new_map):
        # type (OccupancyGrid)-> None
        self.map = new_map
        # a tuple stores the indices of the non-occupied spaces. format ((y,....),(x,...)
        self._free_space_indices = generate_freespace_indices(self.map)


    # def remove_obstacle(self, id):
    #     # type: (int) -> None
    #     del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    #     del_model_prox(str(id))

    def remove_all_obstacles(self, N_OBS):
        # type: (int) -> None
        self.pedsim_manager = PedsimManager()
        self.pedsim_manager.removeAllPeds()
        for obstale in range(N_OBS):
            del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            del_model_prox(str(obstale + 1))

    def spawn_static_obstacle(self, number, pos, radius):
        # type: (list, list, list) -> None
        spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        self.pedsim_manager = PedsimManager()

        # loading the model and setting radius 
        for o_pos_, id, r in zip(pos, number, radius): 
            shape = choice(['cylinder', 'cuboid'])
            s_obs_model_file = rospkg.RosPack().get_path('simulator_setup') + '/obstacles/' + shape + '/model.sdf'
            with open(s_obs_model_file) as f:
                xml_model = f.read()
            xml_model = xml_model.replace('shape', str(r))
            o_pos = Pose(Point(*o_pos_, 1.1), Quaternion(*STANDART_ORIENTATION))
            # spawn obstacle in gazebo & pedsim
            spawn_model("s_obs_%d" % id, xml_model,'', o_pos, 'world')
            # self.pedsim_manager.spawnObstacle(o_pos_, r) # TODO not yet working

    def register_random_dynamic_obstacles(self, num_obstacles, forbidden_zones = None, min_dist=1): 
        # type: (int, list, int) -> None
        
        """register random start and goal position of dynamic obstacles (humans)
        Args:
            num_obstacles (int): number of the obstacles.

        """
        s_pos, g_pos,ids = [],  [], []
        if forbidden_zones == None: forbidden_zones = []

        def dist(x1, y1, x2, y2):
            return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

        for obstacle in range(num_obstacles):
            max_try_times = 20
            i_try = 0
        
            while i_try < max_try_times:
                start_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)
                goal_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)

                if dist(start_pos.position.x, start_pos.position.y, goal_pos.position.x, goal_pos.position.y) < min_dist: 
                    i_try += 1
                    continue
                try:
                    ids.append(obstacle)
                    s_pos.append([start_pos.position.x, start_pos.position.y])
                    g_pos.append([goal_pos.position.x, goal_pos.position.y])
                    # spawn the obstacle with this coordinates and waypoint
                    forbidden_zones.append((start_pos.position.x, start_pos.position.y, self.OBSTACLE_RADIUS))
                    break
                except rospy.ServiceException:
                    i_try += 1
        if i_try == max_try_times:
            # TODO Define specific type of Exception
            raise rospy.ServiceException(
                "can not generate a path with the given start position and the goal position of the robot")
        # load the peds in pedsim format
        print(s_pos,g_pos)
        self.scenario = ArenaScenario()
        self.scenario.createSimplePed(ids, s_pos, g_pos)
        # setup pedsim agents
        self.pedsim_manager = None

        if len(self.scenario.pedsimAgents) > 0:
            self.pedsim_manager = PedsimManager()
            peds = [agent.getPedMsg() for agent in self.scenario.pedsimAgents]
            self.pedsim_manager.spawnPeds(peds)
        return forbidden_zones


    def register_random_static_obstacles(self, num_obstacles, forbidden_zones = None): 
        # type: (int, list) -> list
        """register dynamic obstacles (humans) with random start positions
        Args:
            num_obstacles (int): number of the obstacles.

        """
        status = rospy.get_param('~world')
        # status='no'
        if not status == 'outside':
            forbidden_zones = None
            return forbidden_zones

        pos, ids, radius = [], [], []

        if forbidden_zones == None: forbidden_zones = []
        self.pedsim_manager = PedsimManager()

        for obstacle in range(num_obstacles):

            max_try_times = 20
            i_try = 0
        
            while i_try < max_try_times:
                r = uniform(0.5, 3.)
                pos_obs = get_random_pos_on_map(self._free_space_indices, self.map, r, forbidden_zones)
                try:
                    radius.append(r)
                    ids.append(obstacle)
                    pos.append([pos_obs.position.x, pos_obs.position.y])
                    forbidden_zones.append((pos_obs.position.x, pos_obs.position.y, r))
                    break
                except rospy.ServiceException:
                    i_try += 1
        self.spawn_static_obstacle(ids, pos, radius)
        return forbidden_zones

    def reset_pos_obstacles_random(self, forbidden_zones = None):
            # type: (list) -> None
            elements = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldPropertis)
            for ped in range(elements - 3): # TODO how to get the current Agents?
                start_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)
                goal_pos = get_random_pos_on_map(self._free_space_indices, self.map, 0.2, forbidden_zones)
            # IDEA: 1. find all obstacles; 2.  for every obstacle call the move ped service