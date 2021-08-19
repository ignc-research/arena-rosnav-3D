#!/usr/bin/env python


import rospy
from std_srvs.srv import Trigger
import subprocess
from .ped_manager.ArenaScenario import *
from std_srvs.srv import Trigger
from pedsim_srvs.srv import SpawnPeds, SpawnInteractiveObstacles, MovePeds


class PedsimManager():
    def __init__(self):
        # spawn peds
        spawn_peds_service_name = "pedsim_simulator/spawn_peds"
        rospy.wait_for_service(spawn_peds_service_name, 6.0)
        self.spawn_peds_client = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)
        # respawn peds
        respawn_peds_service_name = "pedsim_simulator/respawn_peds"
        rospy.wait_for_service(respawn_peds_service_name, 6.0)
        self.respawn_peds_client = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)
        # spawn interactive obstacles
        pawn_interactive_obstacles_service_name = "pedsim_simulator/spawn_interactive_obstacles"
        rospy.wait_for_service(pawn_interactive_obstacles_service_name, 6.0)
        self.spawn_interactive_obstacles_client = rospy.ServiceProxy(pawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)
        # respawn interactive obstacles
        respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
        rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
        self.respawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)
        # respawn interactive obstacles
        reset_all_peds_service_name = "pedsim_simulator/reset_all_peds"
        rospy.wait_for_service(reset_all_peds_service_name, 6.0)
        self.reset_all_peds_client = rospy.ServiceProxy(reset_all_peds_service_name, Trigger)
        # remove all peds
        remove_all_peds = "/pedsim_simulator/remove_all_peds"
        rospy.wait_for_service(remove_all_peds, 6.0)
        self.remove_all_peds_client = rospy.ServiceProxy(remove_all_peds, Trigger)
        # move all (dynamic) peds
        move_peds = '/pedsim_simulator/move_peds'
        rospy.wait_for_service(remove_all_peds, 6.0)
        self.move_peds_client = rospy.ServiceProxy(move_peds, MovePeds)

    def spawnPeds(self, peds):
        # type (List[Ped])
        res = self.spawn_peds_client.call(peds)
        #spawn_object_gazebo()
        subprocess.call('rosrun pedsim_gazebo_plugin spawn_pedsim_agents.py', shell = True)
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
        res = self.remove_all_peds_client.call()
        print(res) 

    def setup_spawn_ped(self, number):
        
        self.scenario = ArenaScenario()
        # sample moving object
        scenario_path = RosPack().get_path('simulator_setup') + '/scenarios/empty_map.json'
        self.scenario.loadFromFile(scenario_path)

        # setup pedsim agents
        self.pedsim_manager = None
        if len(self.scenario.pedsimAgents) > 0:
            self.pedsim_manager = PedsimManager()
            ped = [agent.getPedMsg() for agent in self.scenario.pedsimAgents] # ToDo select only the one dynamic agent
            spawnPeds(ped)       

    def move_peds(self):
        res = self.move_peds_client.call()
        print(res) 

