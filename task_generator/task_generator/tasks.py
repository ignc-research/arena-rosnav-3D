#!/usr/bin/env python


import json
import signal
import subprocess
from random import randint, randrange
import six
import abc
import csv
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

    def add_data_to_dataset(self, count, OBS, start_pos, goal_pos):
        """Add random parameters to dataset:
        Args:
            simulation_params
        """
        dir_path = os.path.dirname(os.path.abspath(__file__))
        if count == 1:
            with open(dir_path+"/datagen.csv", "w+", newline="") as file:
                writer = csv.writer(file, delimiter=',')
                header = [["episode", "n_st_obs", "n_dyn_obs",
                           "obs_speed", 'start_pos', 'goal_pos']]
                writer.writerows(header)
                file.close()

        data = np.array([count, OBS['static'], OBS['dynamic'], OBS['speed_dyn'], [
                        start_pos.position.x, start_pos.position.y], [goal_pos.position.x, goal_pos.position.y]])
        print(data)
        print(data.reshape(1, -1))
        with open(dir_path+"/datagen.csv", "a+", newline="") as file:
            # writer has to be defined again for the code to work
            writer = csv.writer(file, delimiter=',')
            writer.writerows(data.reshape(1, -1))  # reshape into line vector
            file.close()

    def _close_gazebo(self):

        # Kill gzclient, gzserver and roscore
        tmp = os.popen("ps -Af").read()
        gzclient_count = tmp.count("gzclient")
        gzserver_count = tmp.count("gzserver")
        # roscore_count = tmp.count("roscore")
        # rosmaster_count = tmp.count("rosmaster")

        if gzclient_count > 0:
            os.system("killall -9 gzclient")
        if gzserver_count > 0:
            os.system("killall -9 gzserver")
        # if rosmaster_count > 0:
        #     os.system("killall -9 rosmaster")
        # if roscore_count > 0:
        #     os.system("killall -9 roscore")

        if gzclient_count or gzserver_count  > 0:
            os.wait()

    def restart_simulation(self, n_dynamic_obstacles: int = 0):
        """restarts the simulation with the needed number of dynamic obstacles written into the world file"""

        rospy.set_param("actors", n_dynamic_obstacles)
        print('One \t 1')
        # process = subprocess.call("killall -q gzclient & killall -q gzserver", shell=True, preexec_fn=os.setsid)
        rospy.sleep(2)
        print(f'One \t 2{n_dynamic_obstacles}')
        self._close_gazebo()
        print('One \t 3')
        rospy.sleep(10)
        world, model = rospy.get_param("world"), rospy.get_param("model")
        rospy.sleep(2)
        # subprocess.Popen(f"roslaunch arena_bringup gazebo_simulator.launch world:={world} model:={model}", shell=True)
        print('One \t 4')
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.sleep(1)


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
                        # self.obstacle_manager.remove_all_obstacles()
                        OBS = {
                            "static": randint(0, 20),
                            "dynamic": randint(0, rospy.get_param('actors', 10)),
                            'speed_dyn': float(randrange(5, 50, 5)/100)
                        }
                        self.restart_simulation(N_OBS["dynamic"])
                        # self.setup_world_params()
                        forbidden_zones = self.obstacle_manager.register_random_static_obstacles(
                            OBS["static"], forbidden_zones=forbidden_zones
                        )
                        self.create_occ_map()
                        (start_pos, goal_pos) = self.robot_manager.set_start_pos_goal_pos(
                            start_pos=S_POS, goal_pos=G_POS)
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
                            ], speed=OBS["speed_dyn"],
                        )
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

    if mode == "manual":
        rospy.set_param("/task_mode", "manual")
        task = ManualTask(pedsim_manager, obstacle_manager, robot_manager)

    return task
