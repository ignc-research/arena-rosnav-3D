#!/usr/bin/env python


import json
import signal
import subprocess
from random import randint, randrange
import six
import abc, csv
import time
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from threading import Lock
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from tf.transformations import quaternion_from_euler
from .robot_manager import RobotManager
from .obstacle_manager import ObstaclesManager
from .pedsim_manager import PedsimManager
from .ped_manager.ArenaScenario import *
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import * 
from threading import Condition, Lock
from filelock import FileLock

STANDART_ORIENTATION = quaternion_from_euler(0.0, 0.0, 0.0)
ROBOT_RADIUS = rospy.get_param("radius")
DATA_GEN = True
if DATA_GEN:
    S_POS = [14.0, 14.0]
    G_POS = [-14.0, -14.0]
count = 0


class StopReset(Exception):
    """Raised when The Task can not be reset anymore"""


@six.add_metaclass(abc.ABCMeta)
class ABSTask(abc.ABCMeta("ABC", (object,), {"__slots__": ()})):
    """An abstract class, all tasks must implement reset function."""

    def __init__(self, pedsim_manager, obstacle_manager, robot_manager):
        # type: (PedsimManager, ObstaclesManager, RobotManager) -> None
        self.robot_manager = robot_manager
        self.obstacle_manager = obstacle_manager
        self.pedsim_manager = pedsim_manager
        self._service_client_get_map = rospy.ServiceProxy(
            "/static_map", GetMap)
        self._map_lock = Lock()
        rospy.Subscriber("/map", OccupancyGrid, self._update_map)
        # a mutex keep the map is not unchanged during reset task.

    @abc.abstractmethod
    def reset(self):
        """
        a function to reset the task. Make sure that _map_lock is used.
        """

    def _update_map(self, map_):
        # type (OccupancyGrid) -> None
        with self._map_lock:
            self.obstacle_manager.update_map(map_)
            self.robot_manager.update_map(map_)


class RandomTask(ABSTask):
    """Evertime the start position and end position of the robot is reset."""

    def __init__(self, pedsim_manager, obstacle_manager, robot_manager):
        # type: (ObstaclesManager, RobotManager, list) -> None
        super(RandomTask, self).__init__(
            pedsim_manager, obstacle_manager, robot_manager
        )
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        if DATA_GEN:
            self.num_of_actors = N_OBS["dynamic"]
        else:
            self.num_of_actors = rospy.get_param("~actors", N_OBS["dynamic"])
        self.unpause()

    def create_occ_map(self):
        """ This function creates an occupancy map of the world this is used for the performance-prediction dataset. (Note the pgm is not added to the navigation stack, the global plan will use the original empty map.pgm)
        """
        # create occupancy map
        global count
        os.makedirs(
            '/home/elias/catkin_ws/src/arena-rosnav-3D/occ_maps', exist_ok=True)
        os.chdir('/home/elias/catkin_ws/src/arena-rosnav-3D/occ_maps')
        print('succsess')
        subprocess.call(
            'rosservice call /gazebo_2Dmap_plugin/generate_map', shell=True, preexec_fn=os.setsid)
        # time.sleep(10)
        subprocess.call(
            f'rosrun map_server map_saver -f map_{count} /map:=/map2d', shell=True, preexec_fn=os.setsid)
        time.sleep(1)
        count += 1
    
    def setup_world_params():
        """This method can be used to alter world elements like the robot urdf, this can be helpfull for changing for example the laser range.
        This method can be used to alter the navigation stack. It will require you to shut down the node and restart is afterwards"""
        # 1. stop Gazebo sim (leave-roscore running, only close the one element)
        # 2. load the xml and alter the params
        # 3. restart Gazebo
        # (4. Add the params to the dataset)
        raise NotImplemented

    def add_data_to_dataset(count, OBS, start_pos, goal_pos):
        """Add random parameters to dataset:
        Args:
            simulation_params
        """
        dir_path = os.path.dirname(os.path.abspath(__file__))
        if count == 0:
            with open(dir_path+"/datagen.csv", "w+", newline = "") as file:
                writer = csv.writer(file, delimiter = ',')
                header = [["episode","n_st_obs","n_dyn_obs","obs_speed", 'start_pos', 'goal_pos']]
                writer.writerows(header)
                file.close()

        data = np.array(count, OBS['static'], OBS['dynamic'], OBS['speed_dyn'], start_pos, goal_pos)
        with open(dir_path+"/{0}_{1}--{2}--{3}.csv"), "a+", newline = "") as file:
            writer = csv.writer(file, delimiter = ',') # writer has to be defined again for the code to work
            writer.writerows(data.reshape(1,-1)) # reshape into line vector
            file.close()        


    def reset(self):
        """[summary]"""
        info = {}
        forbidden_zones = []
        with self._map_lock:
            max_fail_times = 3
            fail_times = 0

            while fail_times < max_fail_times:
                try:
                    print("loglog: reached goal 2")

                    if DATA_GEN:
                        self.obstacle_manager.remove_all_obstacles() 
                        OBS = {
                            "static": randint(0, 20),
                            "dynamic": randint(0, rospy.get_param('actors', 10)),
                            'speed_dyn': float(randrange(5,50,5)/100)
                        }
                        # self.setup_world_params()
                        forbidden_zones = self.obstacle_manager.register_random_static_obstacles(
                            OBS["static"], forbidden_zones=forbidden_zones
                        )
                        self.create_occ_map()
                        self.obstacle_manager.register_random_dynamic_obstacles(
                        OBS["dynamic"],
                        forbidden_zones=[
                            (
                                start_pos.position.x,
                                start_pos.position.y,
                                ROBOT_RADIUS,
                            ),
                            (
                                goal_pos.position.x,
                                goal_pos.position.y,
                                ROBOT_RADIUS,
                            ),
                        ], speed = OBS["speed_dyn"],
                    )

                    (
                        start_pos,
                        goal_pos,
                    ) = self.robot_manager.set_start_pos_goal_pos(start_pos=S_POS, goal_pos=G_POS)
                     # removes all peds
                    self.add_data_to_dataset(count, OBS, start_pos, goal_pos)
                    print("loglog: reached goal 3")
                    break
                except rospy.ServiceException as e:
                    rospy.logwarn(repr(e))
                    fail_times += 1
            if fail_times == max_fail_times:
                raise Exception("reset error!")
            info["robot_goal_pos"] = np.array(
                [goal_pos.position.x, goal_pos.position.y]
            )
        return info


class ManualTask(ABSTask):
    """randomly spawn obstacles and user can manually set the goal postion of the robot"""

    def __init__(self, pedsim_manager, obstacle_manager, robot_manager):
        # type: (ObstaclesManager, RobotManager, list) -> None
        super(ManualTask, self).__init__(
            pedsim_manager, obstacle_manager, robot_manager
        )
        # subscribe
        rospy.Subscriber("/manual_goal", PoseStamped, self._set_goal_callback)
        self._goal = PoseStamped()
        self._new_goal_received = False
        self._manual_goal_con = Condition()
        self.num_of_actors = rospy.get_param("~actors", 3)

    def is_True(self):
        if self._new_goal_received is True:
            return True
        else:
            return False

    def reset(self):
        while True:
            with self._map_lock:
                self.obstacle_manager.remove_all_obstacles(N_OBS["static"])
                start_pos = self.robot_manager.set_start_pos_random()
                self.obstacle_manager.register_random_dynamic_obstacles(
                    self.num_of_actors,
                    forbidden_zones=[
                        (
                            start_pos.position.x,
                            start_pos.position.y,
                            ROBOT_RADIUS,
                        ),
                    ],
                )
                with self._manual_goal_con:
                    # the user has 60s to set the goal, otherwise all objects will be reset.
                    self._manual_goal_con.wait_for(
                        lambda: self.is_True() is True, timeout=60
                    )
                    if not self._new_goal_received:
                        raise Exception(
                            "TimeOut, User does't provide goal position!")
                    else:
                        self._new_goal_received = False
                    try:
                        # in this step, the validation of the path will be checked
                        self.robot_manager._goal_pub.publish(self._goal)
                        self.robot_manager.pub_mvb_goal.publish(self._goal)

                    except Exception as e:
                        rospy.logwarn(repr(e))

    def _set_goal_callback(self, goal: PoseStamped):
        with self._manual_goal_con:
            self._goal = goal
            self._new_goal_received = True
            self._manual_goal_con.notify()


# class StagedRandomTask(RandomTask):
#     def __init__(
#         self,
#         ns,
#         pedsim_manager,
#         obstacle_manager,
#         robot_manager,
#         start_stage=1,
#         PATHS=None,
#     ):
#         # type: (str, PedsimManager, ObstaclesManager, RobotManager, int, dict) -> None
#         super(StagedRandomTask, self).__init__(
#             pedsim_manager, obstacle_manager, robot_manager
#         )
#         self.ns = ns
#         self.ns_prefix = "" if ns == "" else "/" + ns + "/"

#         self._curr_stage = start_stage
#         self._stages = {}
#         self._PATHS = PATHS
#         self._read_stages_from_yaml()

#         rospy.set_param("/task_mode", "staged")

#         # check start stage format
#         if not isinstance(start_stage, int):
#             raise ValueError("Given start_stage not an Integer!")
#         if self._curr_stage < 1 or self._curr_stage > len(self._stages):
#             raise IndexError(
#                 "Start stage given for training curriculum out of bounds! Has to be between {1 to %d}!"
#                 % len(self._stages)
#             )
#         rospy.set_param("/curr_stage", self._curr_stage)

#         # hyperparamters.json location
#         self.json_file = os.path.join(
#             self._PATHS.get("model"), "hyperparameters.json")
#         assert os.path.isfile(self.json_file), (
#             "Found no 'hyperparameters.json' at %s" % self.json_file
#         )
#         self._lock_json = FileLock(self.json_file + ".lock")

#         # subs for triggers
#         self._sub_next = rospy.Subscriber(
#             f"{self.ns_prefix}next_stage", Bool, self.next_stage
#         )
#         self._sub_previous = rospy.Subscriber(
#             f"{self.ns_prefix}previous_stage", Bool, self.previous_stage
#         )

#         self._initiate_stage()

#     def next_stage(self, msg):
#         # type (Bool) -> Any
#         if self._curr_stage < len(self._stages):
#             self._curr_stage = self._curr_stage + 1
#             self._initiate_stage()

#             if self.ns == "eval_sim":
#                 rospy.set_param("/curr_stage", self._curr_stage)
#                 with self._lock_json:
#                     self._update_curr_stage_json()

#                 if self._curr_stage == len(self._stages):
#                     rospy.set_param("/last_stage_reached", True)
#         else:
#             print(
#                 f"({self.ns}) INFO: Tried to trigger next stage but already reached last one"
#             )

#     def previous_stage(self, msg):
#         # type (Bool) -> Any
#         if self._curr_stage > 1:
#             rospy.set_param("/last_stage_reached", False)

#             self._curr_stage = self._curr_stage - 1
#             self._initiate_stage()

#             if self.ns == "eval_sim":
#                 rospy.set_param("/curr_stage", self._curr_stage)
#                 with self._lock_json:
#                     self._update_curr_stage_json()
#         else:
#             print(
#                 f"({self.ns}) INFO: Tried to trigger previous stage but already reached first one"
#             )

#     def _initiate_stage(self):
#         self.obstacle_manager.remove_all_obstacles()
#         n_dynamic_obstacles = self._stages[self._curr_stage]["dynamic"]

#         print(f"num_dynamic obs {n_dynamic_obstacles}")
#         print(f'num_static obs {self._stages[self._curr_stage]["static"]}')

#         # When additional actors need to be loaded into the simulation, a new world file is created & gazebo restarted
#         if (
#             self._curr_stage == 1
#             or n_dynamic_obstacles != self._stages[self._curr_stage - 1]["dynamic"]
#         ):
#             rospy.set_param("actors", n_dynamic_obstacles)
#             subprocess.call(
#                 "killall -q gzclient & killall -q gzserver", shell=True)
#             subprocess.call(
#                 "rosrun task_generator generate_world.py", shell=True)
#             world, model = rospy.get_param("world"), rospy.get_param("model")
#             subprocess.Popen(
#                 f"roslaunch arena_bringup gazebo_simulator.launch world:={world} model:={model}",
#                 shell=True,
#             )
#             rospy.wait_for_service("/gazebo/spawn_urdf_model")
#             self.robot_manager.spawn_robot()
#             rospy.sleep(10)

#         else:
#             self.obstacle_manager.register_random_dynamic_obstacles(
#                 n_dynamic_obstacles)

#         print(
#             f"({self.ns}) Stage {self._curr_stage}: Spawning {n_dynamic_obstacles} dynamic obstacles!"
#         )

#     def _read_stages_from_yaml(self):
#         file_location = self._PATHS.get("curriculum")
#         if os.path.isfile(file_location):
#             with open(file_location, "r") as file:
#                 self._stages = yaml.load(file, Loader=yaml.FullLoader)
#             assert isinstance(
#                 self._stages, dict
#             ), "'training_curriculum.yaml' has wrong fromat! Has to encode dictionary!"
#         else:
#             raise FileNotFoundError(
#                 "Couldn't find 'training_curriculum.yaml' in %s "
#                 % self._PATHS.get("curriculum")
#             )

#     def _update_curr_stage_json(self):
#         with open(self.json_file, "r") as file:
#             hyperparams = json.load(file)
#         try:
#             hyperparams["curr_stage"] = self._curr_stage
#         except Exception as e:
#             raise Warning(
#                 f" {e} \n Parameter 'curr_stage' not found in 'hyperparameters.json'!"
#             )
#         else:
#             with open(self.json_file, "w", encoding="utf-8") as target:
#                 json.dump(hyperparams, target, ensure_ascii=False, indent=4)


# class ScenarioTask(ABSTask):
#     def __init__(self, pedsim_manager, obstacle_manager, robot_manager, scenario_path):
#         # type: (PedsimManager, ObstaclesManager, RobotManager, str) -> None
#         super(ScenarioTask, self).__init__(
#             pedsim_manager, obstacle_manager, robot_manager
#         )

#         # load scenario from file
#         self.scenario = ArenaScenario()
#         self.scenario.loadFromFile(scenario_path)

#         # setup pedsim agents
#         self.pedsim_manager = None
#         # if len(self.scenario.pedsimAgents) > 0:
#         #     self.pedsim_manager = pedsim_manager
#         #     peds = [agent.getPedMsg() for agent in self.scenario.pedsimAgents]
#         #     self.pedsim_manager.spawnPeds(peds)
#         # self.reset_count = 0

    # def reset(self):
    #     if self.scenario.resets >= self.reset_count:
    #         self.reset_count += 1
    #         info = {}
    #         with self._map_lock:
    #             # reset pedsim agents
    #             # if self.pedsim_manager != None:
    #             #     self.pedsim_manager.resetAllPeds()

    #             # reset robot
    #             self.robot_manager.set_start_pos_goal_pos(
    #                 Pose(
    #                     Point(*np.append(self.scenario.robotPosition, 0)),
    #                     Quaternion(*STANDART_ORIENTATION),
    #                 ),
    #                 Pose(
    #                     Point(*np.append(self.scenario.robotGoal, 0)),
    #                     Quaternion(*STANDART_ORIENTATION),
    #                 ),
    #             )

    #             # fill info dict
    #             if self.reset_count == 1:
    #                 info["new_scenerio_loaded"] = True
    #             else:
    #                 info["new_scenerio_loaded"] = False
    #             info["robot_goal_pos"] = self.scenario.robotGoal
    #             info["num_repeats_curr_scene"] = self.reset_count
    #             info[
    #                 "max_repeats_curr_scene"
    #             ] = 1000  # TODO: implement max number of repeats for scenario
    #         return info

    #     else:
    #         return "End"


def get_predefined_task(ns, mode="random", start_stage=1, PATHS=None):
    # type: (str, str, int, dict) -> None

    # get the map
    service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)
    map_response = service_client_get_map()

    robot_manager = RobotManager(ns="", map_=map_response.map)
    obstacle_manager = ObstaclesManager(ns="", map_=map_response.map)
    pedsim_manager = PedsimManager()

    global N_OBS
    # ! Make sure to have set 'actors'-param in the launch file to 20, to include 20 actor elements in the world file
    N_OBS = {
        "static": randint(0, 20),
        "dynamic": randint(0, 20),
    }
    if DATA_GEN:
        forbidden_zones = [(*S_POS, .2), (*G_POS, .2), (0, 0, .2)]

# Todo:
# - Add map pluin subrocess call (for loop map name++)
# - Add random dyn obs ability
# - check center of random world in middle of the world
# - Add robot start pos to forbidden zones
# - create new map file

    # Tasks will be moved to other classes or functions.
    task = None
    if mode == "random":

        if DATA_GEN:
            rospy.set_param("/task_mode", "random")

            # forbidden_zones = obstacle_manager.register_random_dynamic_obstacles(
            #     N_OBS["dynamic"], forbidden_zones=forbidden_zones
            # )
            task = RandomTask(pedsim_manager, obstacle_manager, robot_manager)
            print("random tasks requested")
        else:
            rospy.set_param("/task_mode", "random")
            forbidden_zones = obstacle_manager.register_random_static_obstacles(
                N_OBS["static"]
            )
            forbidden_zones = obstacle_manager.register_random_dynamic_obstacles(
                N_OBS["dynamic"], forbidden_zones=forbidden_zones
            )
            task = RandomTask(pedsim_manager, obstacle_manager, robot_manager)
            print("random tasks requested")
    # if mode == "staged":
    #     rospy.set_param("/task_mode", "staged")
    #     task = StagedRandomTask(
    #         ns, pedsim_manager, obstacle_manager, robot_manager, start_stage, PATHS
    #     )

    # if mode == "scenario":
    #     rospy.set_param("/task_mode", "scenario")
    #     # forbidden_zones = obstacle_manager.register_random_static_obstacles(
    #     #     N_OBS["static"]
    #     # )
    #     task = ScenarioTask(
    #         pedsim_manager, obstacle_manager, robot_manager, PATHS["scenario"]
    #     )
    # if mode == "scenario_staged":
    #     rospy.set_param("/task_mode", "scenario_staged")
    #     task = ScenarioTask(
    #         pedsim_manager, obstacle_manager, robot_manager, PATHS["scenario"]
    #     )

    if mode == "manual":
        rospy.set_param("/task_mode", "manual")
        task = ManualTask(pedsim_manager, obstacle_manager, robot_manager)

    return task
