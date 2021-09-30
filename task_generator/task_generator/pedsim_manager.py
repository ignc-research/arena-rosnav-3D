#!/usr/bin/env python


import rospy
import itertools
from std_srvs.srv import Trigger
from std_msgs.msg import Header
import subprocess
from .ped_manager.ArenaScenario import *
from std_srvs.srv import Trigger, SetBool
from pedsim_srvs.srv import SpawnPeds, SpawnInteractiveObstacles, MovePeds, SpawnObstacle, SetObstacles
from geometry_msgs.msg import Point
from pedsim_msgs.msg import LineObstacles, LineObstacle


class PedsimManager():
    def __init__(self):
        # spawn peds
        spawn_peds_service_name = "pedsim_simulator/spawn_peds"
        rospy.wait_for_service(spawn_peds_service_name, 6.0)
        self.spawn_peds_client = rospy.ServiceProxy(
            spawn_peds_service_name, SpawnPeds)
        # respawn peds
        respawn_peds_service_name = "pedsim_simulator/respawn_peds"
        rospy.wait_for_service(respawn_peds_service_name, 6.0)
        self.respawn_peds_client = rospy.ServiceProxy(
            respawn_peds_service_name, SpawnPeds)
        # spawn interactive obstacles
        pawn_interactive_obstacles_service_name = "pedsim_simulator/spawn_interactive_obstacles"
        rospy.wait_for_service(pawn_interactive_obstacles_service_name, 6.0)
        self.spawn_interactive_obstacles_client = rospy.ServiceProxy(
            pawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)
        # respawn interactive obstacles
        respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
        rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
        self.respawn_interactive_obstacles_client = rospy.ServiceProxy(
            respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)
        # respawn interactive obstacles
        reset_all_peds_service_name = "pedsim_simulator/reset_all_peds"
        rospy.wait_for_service(reset_all_peds_service_name, 6.0)
        self.reset_all_peds_client = rospy.ServiceProxy(
            reset_all_peds_service_name, Trigger)
        # remove all peds
        remove_all_peds = "/pedsim_simulator/remove_all_peds"
        rospy.wait_for_service(remove_all_peds, 6.0)
        self.remove_all_peds_client = rospy.ServiceProxy(
            remove_all_peds, SetBool)
        # move all (dynamic) peds
        move_peds = '/pedsim_simulator/move_peds'
        rospy.wait_for_service(remove_all_peds, 6.0)
        self.move_peds_client = rospy.ServiceProxy(move_peds, MovePeds)
        # spawn Object in pedsim
        spawn_obstacle = '/pedsim_simulator/add_obstacle'
        rospy. wait_for_service(spawn_obstacle, 6.0)
        self.spawn_obstacle = rospy.ServiceProxy(spawn_obstacle, SpawnObstacle)
        # set scene obstacles based on ped_scenario file_location
        set_obstacles = '/pedsim_simulator/set_obstacles'
        rospy.wait_for_service(set_obstacles, 6.0)
        self.set_obstacles_client = rospy.ServiceProxy(
            set_obstacles, SetObstacles)

    def spawnPeds(self, peds):
        # type (List[Ped])
        res = self.spawn_peds_client.call(peds)
        print(res)

    def respawnPeds(self, peds):
        # type (List[Ped])
        res = self.respawn_peds_client.call(peds)
        print(res)

    def spawnInteractiveObstacles(self, obstacles):
        # type (List[InteractiveObstacle])
        res = self.spawn_interactive_obstacles_client.call(obstacles)
        print(res)

    def respawnInteractiveObstacles(self, obstacles):
        # type (List[InteractiveObstacle])
        res = self.respawn_interactive_obstacles_client.call(obstacles)
        print(res)

        # setting peds in initial position
    def resetAllPeds(self):
        res = self.reset_all_peds_client.call()
        print(res)

    def removeAllPeds(self):
        res = self.remove_all_peds_client.call(True)
        print()

    def move_peds(self):
        res = self.move_peds_client.call()
        print(res)

    # This does not yet work. the LineObstacle message is not received by pedsim yet !
    def spawnObstacle(self, position, size):
        # type: (list, float) -> None
        start_x = position[0] - size/2
        start_y = position[1] - size/2
        end_x = position[0] + size/2
        end_y = position[1] + size/2

        # creates the coordinates of the linear obstacles, by combining uniquely combining all four corner coordinates (in total 4 obstacles are spawned)
        pos = list(itertools.product([start_x, start_y], [end_x, end_y]))
        # for a, b  in zip(pos[::2], pos[1::2]):
        #     self.spawn_obstacle(LineObstacles(header = Header(), obstacle = LineObstacle(start = Point(*a, 0), end = Point(*b, 0))))

    def setObstacles(self, map_name):
        res = self.set_obstacles_client.call(map_name)
        print(res)
