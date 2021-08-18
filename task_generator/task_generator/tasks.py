#!/usr/bin/env python


import json, six, abc, actionlib
import rospy, math, time, random
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Pose2D
from threading import Lock, Condition
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int16
import numpy as np
from tf.transformations import quaternion_from_euler
from pathlib import Path
from std_srvs.srv import Trigger
import rospkg, subprocess
from .robot_manager import RobotManager
from .obstacle_manager import ObstaclesManager
from .ped_manager.ArenaScenario import *
from std_srvs.srv import Trigger
from pedsim_srvs.srv import SpawnPeds, SpawnInteractiveObstacles, SpawnObstacle, MovePeds
from pedsim_msgs.msg import Ped, AgentStates, InteractiveObstacle, LineObstacles
from pedsim_msgs.msg import LineObstacle


standart_orientation = quaternion_from_euler(0.0,0.0,0.0)
ROBOT_RADIUS = 0.17


from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from rospkg import RosPack
import roslibpy

global xml_file




def spawn_object_gazebo():
    """ ToDo
    PROBLEM: the subscriber callback function is only called when using rospy.spin
        -> if not solved agents are not spawned
        (Description: see https://levelup.gitconnected.com/ros-spinning-threading-queuing-aac9c0a793f (aperently the queue is not called (It is called (and the obstacles are first spawned) when the rospy.Publisher is first called (in robot manager line 61))))
    """
    rospy.sleep(5)
    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
    default_actor_model_file = pkg_path + "/models/actor_model.sdf"

    actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
    file_xml = open(actor_model_file)
    xml_string = file_xml.read()

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    print("service: spawn_sdf_model is available ....")

    def actor_poses_callback(actors):
        for actor in actors.agent_states:
            actor_id = str( actor.id )
            actor_pose = actor.pose
            rospy.loginfo("Spawning model: actor_id = %s", actor_id)

            model_pose = Pose(Point(x= actor_pose.position.x,
                                y= actor_pose.position.y,
                                z= actor_pose.position.z),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )
                        # model_name # model_xml # robot_namespace # pose # reference_frame -> of Spawn Model
            spawn_model(actor_id, xml_string, "", model_pose, "world")
        sub.unregister()

    sub = rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, actor_poses_callback)



class StopReset(Exception):
    """Raised when The Task can not be reset anymore """


@six.add_metaclass(abc.ABCMeta)
class ABSTask(abc.ABCMeta('ABC', (object,), {'__slots__': ()})):
    """An abstract class, all tasks must implement reset function.
    """


    def __init__(self, obstacle_manager, robot_manager):
        # type: (ObstaclesManager, RobotManager) -> None
        #self.obstacles_manager = obstacles_manager
        self.robot_manager = robot_manager
        self.obstacle_manager = obstacle_manager
        self._service_client_get_map = rospy.ServiceProxy('/static_map', GetMap)
        self._map_lock = Lock()
        rospy.Subscriber('/map', OccupancyGrid, self._update_map)
        # a mutex keep the map is not unchanged during reset task.

    @abc.abstractmethod #abstract methods must be implemented in its sub-classes
    def reset(self):
        """
        a funciton to reset the task. Make sure that _map_lock is used.
        """

    def _update_map(self, map_):
        # type (OccupancyGrid) -> None
        with self._map_lock:
            self.obstacle_manager.update_map(map_)
            self.robot_manager.update_map(map_)


class RandomTask(ABSTask):
    """ Evertime the start position and end position of the robot is reset.
    """


    def __init__(self, obstacle_manager, robot_manager):
        #type: (ObstaclesManager, RobotManager) -> None
        super(RandomTask, self).__init__(obstacle_manager, robot_manager)

    def reset(self):
        """[summary]
        """
        with self._map_lock:
            max_fail_times = 3
            fail_times = 0
            while fail_times < max_fail_times:
                try:
                    start_pos, goal_pos = self.robot_manager.set_start_pos_goal_pos()
                    self.obstacle_manager.reset_pos_obstacles_random(
                        forbidden_zones=[
                            (start_pos.position.x, start_pos.position.y, ROBOT_RADIUS),
                            (goal_pos.position.x, goal_pos.position.y, ROBOT_RADIUS)])
                    break
                except rospy.ServiceException as e:
                    rospy.logwarn(repr(e))
                    fail_times += 1
            if fail_times == max_fail_times:
                raise Exception("reset error!")


class ManualTask(ABSTask):
    """randomly spawn obstacles and user can mannually set the goal postion of the robot
    """


    def __init__(self, ns, obstacle_manager, robot_manager):##################################################### ToDo, include obstacles
        # type: (str, ObstaclesManager, RobotManager) -> Any
        super(ManualTask, self).__init__(robot_manager)
        self.ns = ns
        self.ns_prefix = "" if ns == '' else "/"+ns+"/"
        # subscribe
        rospy.Subscriber('%smanual_goal' % self.ns, Pose2D, self._set_goal_callback)
        self._goal = Pose2D()
        self._new_goal_received = False
        self._manual_goal_con = Condition()

    def reset(self):
        while True:
            with self._map_lock:
                #self.obstacles_manager.reset_pos_obstacles_random()
                self.robot_manager.set_start_pos_random()
                with self._manual_goal_con:
                    # the user has 60s to set the goal, otherwise all objects will be reset.
                    self._manual_goal_con.wait_for(
                        self._new_goal_received, timeout=60)
                    if not self._new_goal_received:
                        raise Exception(
                            "TimeOut, User does't provide goal position!")
                    else:
                        self._new_goal_received = False
                    try:
                        # in this step, the validation of the path will be checked
                        self.robot_manager.publish_go_new_goal_receivedal(
                            Pose(Point(self._goal.x, self._goal.y, 0), Quaternion(quaternion_from_euler(0.0,self._goal.theta,0.0))))
                    except rospy.ServiceException as e:
                        rospy.logwarn(repr(e))
                
    def _set_goal_callback(self, goal):
        # type: (Pose) -> None
        with self._manual_goal_con:
            self._goal = goal
            self._new_goal_received = True
        self._manual_goal_con.notify()


# /home/elias/catkin_ws/src/arena-rosnav-3D/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/configs/training_curriculum_map1small.yaml
# see train_agent.py for details
class StagedRandomTask(RandomTask):
    def __init__(self, ns, obstacle_manager, robot_manager, start_stage = 1, PATHS=None):
        # type: (str, ObstaclesManager, RobotManager, int, dict) -> None
        super(StagedRandomTask, self).__init__(obstacle_manager, robot_manager)
        self.ns = ns
        self.ns_prefix = "" if ns == '' else "/"+ns+"/"

        self._curr_stage = start_stage
        self._stages = {}
        self._PATHS = PATHS
        self._read_stages_from_yaml()

        # check start stage format
        if not isinstance(start_stage, int):
            raise ValueError(
                "Given start_stage not an Integer!")
        if (self._curr_stage < 1 or 
            self._curr_stage > len(self._stages)):
            raise IndexError(
                "Start stage given for training curriculum out of bounds! Has to be between {1 to %d}!" % len(self._stages))
        rospy.set_param("/curr_stage", self._curr_stage)

        # hyperparamters.json location
        self.json_file = os.path.join(
            self._PATHS.get('model'), "hyperparameters.json")
        assert os.path.isfile(self.json_file), "Found no 'hyperparameters.json' at %s" % self.json_file
        self._lock_json = FileLock(self.json_file + ".lock")

        # subs for triggers
        self._sub_next = rospy.Subscriber(self.ns_prefix, "next_stage", Bool, self.next_stage)
        self._sub_previous = rospy.Subscriber(self.ns_prefix, "previous_stage", Bool, self.previous_stage)

        self._initiate_stage()

    def next_stage(self, msg):
        # type (Bool) -> Any
        if self._curr_stage < len(self._stages):
            self._curr_stage = self._curr_stage + 1
            self._initiate_stage()

            if self.ns == "eval_sim":
                rospy.set_param("/curr_stage", self._curr_stage)
                with self._lock_json:
                    self._update_curr_stage_json()
                    
                if self._curr_stage == len(self._stages):
                    rospy.set_param("/last_stage_reached", True)
        else:
            print("(", self.ns, ") INFO: Tried to trigger next stage but already reached last one")

    def previous_stage(self, msg):
        # type (Bool) -> Any
        if self._curr_stage > 1:
            rospy.set_param("/last_stage_reached", False)

            self._curr_stage = self._curr_stage - 1
            self._initiate_stage()

            if self.ns == "eval_sim":
                rospy.set_param("/curr_stage", self._curr_stage)
                with self._lock_json:
                    self._update_curr_stage_json()
        else:
            print("(", self.ns, ") INFO: Tried to trigger previous stage but already reached first one")

    def _initiate_stage(self):
        self._remove_obstacles()
        
        n_dynamic_obstacles = self._stages[self._curr_stage]['dynamic']

        self.obstacles_manager.register_random_dynamic_obstacles(n_dynamic_obstacles, forbidden_zones = [(start_pos.x, start_pos.y, ROBOT_RADIUS), (goal_pos.x, goal_pos.y, ROBOT_RADIUS)])

        print("(", self.ns, ") Stage ", self._curr_stage, ": Spawning ", dynamic_obstacles, " dynamic obstacles!")

    def _read_stages_from_yaml(self):
        file_location = self._PATHS.get('curriculum')
        if os.path.isfile(file_location):
            with open(file_location, "r") as file:
                self._stages = yaml.load(file, Loader=yaml.FullLoader)
            assert isinstance(
                self._stages, dict), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
        else:
            raise FileNotFoundError(
                "Couldn't find 'training_curriculum.yaml' in %s " % self._PATHS.get('curriculum'))

    def _update_curr_stage_json(self):
        with open(self.json_file, "r") as file:
            hyperparams = json.load(file)
        try:
            hyperparams['curr_stage'] = self._curr_stage
        except Exception as e:
            raise Warning(
                " ",e, " \n Parameter 'curr_stage' not found in 'hyperparameters.json'!")
        else:
            with open(self.json_file, "w", encoding='utf-8') as target:
                json.dump(hyperparams, target,
                        ensure_ascii=False, indent=4)

    def _remove_obstacles(self):
        # idea rosservice call /pedsim_simulator/remove_all_peds true (to remove all obstacles)
        self.obstacles_manager.remove_obstacles()


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



class ScenarioTask(ABSTask):
    def __init__(self, obstacle_manager, robot_manager, scenario_path):
        # type: (ObstaclesManager, RobotManager, str) -> None
        super(ScenarioTask, self).__init__(obstacle_manager, robot_manager)

        # load scenario from file
        self.scenario = ArenaScenario()
        self.scenario.loadFromFile(scenario_path)

        # setup pedsim agents
        self.pedsim_manager = None
        if len(self.scenario.pedsimAgents) > 0:
            self.pedsim_manager = PedsimManager()
            peds = [agent.getPedMsg() for agent in self.scenario.pedsimAgents]
            self.pedsim_manager.spawnPeds(peds)

       # spawn_peds_in_gazebo()

        self.reset_count = 0

    def reset(self):
        self.reset_count += 1
        info = {}
        with self._map_lock:
            # reset pedsim agents
            if self.pedsim_manager != None:
                self.pedsim_manager.resetAllPeds()

            # reset robot
            self.robot_manager.set_start_pos_goal_pos(
                Pose(Point(*np.append(self.scenario.robotPosition, 0)), Quaternion(*standart_orientation)), 
                Pose(Point(*np.append(self.scenario.robotGoal, 0)), Quaternion(*standart_orientation))
                )

            # fill info dict
            if self.reset_count == 1:
                info["new_scenerio_loaded"] = True
            else:
                info["new_scenerio_loaded"] = False
            info["robot_goal_pos"] = self.scenario.robotGoal
            info['num_repeats_curr_scene'] = self.reset_count
            info['max_repeats_curr_scene'] = 1000  # todo: implement max number of repeats for scenario
        return info


def get_predefined_task(ns, mode="random", start_stage = 1, PATHS = None):
    # type: (str, str, int, dict) -> Any

    # get the map
    service_client_get_map = rospy.ServiceProxy('/static_map', GetMap)
    map_response = service_client_get_map()

    # use rospkg to get the path where the model config yaml file stored
    models_folder_path = rospkg.RosPack().get_path('simulator_setup')

    robot_manager = RobotManager(ns='',map_= map_response.map)
    obstacle_manager = ObstaclesManager(ns='',map_= map_response.map)

    # Tasks will be moved to other classes or functions.
    task = None
    if mode == "random":
        rospy.set_param("/task_mode", "random")
        task = RandomTask(obstacle_manager, robot_manager)
        print("random tasks requested")
    if mode == "manual":
        rospy.set_param("/task_mode", "manual")
        task = ManualTask(ns, robot_manager)
        print("manual tasks requested")
    if mode == "staged":
        rospy.set_param("/task_mode", "staged")
        task = StagedRandomTask(ns, start_stage, PATHS)
    if mode == "scenario":
        rospy.set_param("/task_mode", "scenario")
        task = ScenarioTask(obstacle_manager, robot_manager, PATHS['scenario'])

    return task