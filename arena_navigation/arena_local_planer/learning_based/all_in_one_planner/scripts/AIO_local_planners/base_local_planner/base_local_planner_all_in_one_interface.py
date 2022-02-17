import subprocess
import time

import numpy as np
import rospy
import rosservice

import all_in_one_local_planner_interface.srv
from AIO_local_planners.model_base_class import ModelBase


class BaseLocalPlannerAgent(ModelBase):

    def __init__(self, name: str, config_path: str):
        observation_information = {'robot_twist': True,
                                   'global_plan_raw': True,
                                   'new_global_plan': True}
        super().__init__(observation_information, name)

        # Generate local planner node
        package = 'all_in_one_local_planner_interface'
        launch_file = 'start_local_planner_node.launch'

        arg1 = "ns:=_"
        arg4 = "use_ns:=false"

        arg2 = "node_name:=" + name
        arg3 = "config_path:=" + config_path

        # Use subprocess to execute .launch file
        self._local_planner_process = subprocess.Popen(["roslaunch", package, launch_file, arg1, arg2, arg3, arg4],
                                                       stdout=subprocess.DEVNULL,
                                                       stderr=subprocess.STDOUT)
        # self._local_planner_process = subprocess.Popen(["roslaunch", package, launch_file, arg1, arg2, arg3, arg4])

        self._getVelServiceGlobalPlan = None
        self._getVelService = None
        self._resetCostmapService = None

        self._local_planner_rdy = False

    def get_next_action(self, observation_dict: dict) -> np.ndarray:
        while not self._local_planner_rdy:
            self.wait_for_agent()
            if not self._local_planner_rdy:
                rospy.logwarn("Couldn't find local planner " + self._name + "!")
        if observation_dict['new_global_plan']:

            global_plan = observation_dict['global_plan_raw']
            request_msg = all_in_one_local_planner_interface.srv.GetVelCmdWithGlobalPlanRequest(global_plan=global_plan)
            response_msg: all_in_one_local_planner_interface.srv.GetVelCmdWithGlobalPlanResponse = self._getVelServiceGlobalPlan(
                request_msg)
        else:
            response_msg: all_in_one_local_planner_interface.srv.GetVelCmdResponse = self._getVelService()

        if not response_msg.successful:
            response_msg.vel.linear.x = 0.06
            rospy.logwarn("TEB couldn't find feasable path, set linear velocity to 0.06!")

        if response_msg.costmaps_resetted:
            rospy.logwarn("Local Costmap resetted!")

        return np.array([response_msg.vel.linear.x, response_msg.vel.linear.y, response_msg.vel.angular.z])

    def wait_for_agent(self) -> bool:
        service_name_get_vel_global_plan = "/" + self._name + "/" + "getVelCommandWithGlobalPlan"
        service_name_cmd_vel = "/" + self._name + "/" + "getVelCmd"
        service_name_reset_costmap = "/" + self._name + "/" + "resetCostmap"
        # wait until service is available
        service_list = rosservice.get_service_list()
        max_tries = 10
        for i in range(max_tries):
            if service_name_get_vel_global_plan in service_list and service_name_reset_costmap in service_list \
                    and service_name_cmd_vel in service_list:
                break
            else:
                time.sleep(0.3)

        if service_name_get_vel_global_plan not in service_list:
            self._local_planner_rdy = False
            return False
        else:
            self._getVelServiceGlobalPlan = rospy.ServiceProxy(service_name_get_vel_global_plan,
                                                               all_in_one_local_planner_interface.srv.GetVelCmdWithGlobalPlan,
                                                               persistent=True)
            self._getVelService = rospy.ServiceProxy(service_name_cmd_vel,
                                                     all_in_one_local_planner_interface.srv.GetVelCmd,
                                                     persistent=True)
            self._resetCostmapService = rospy.ServiceProxy(service_name_reset_costmap,
                                                           all_in_one_local_planner_interface.srv.ResetCostmap,
                                                           persistent=True)
            self._local_planner_rdy = True
            return True

    def reset(self):
        """
        Send service request to clear costmap.
        """
        if self._local_planner_rdy:
            self._resetCostmapService()

    def close(self):
        self._local_planner_process.terminate()
