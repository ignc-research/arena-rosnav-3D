#!/usr/bin/env python3

"""This file is used for large scale scenario testing. 
    For usage, refere to the documentation here: TODO
"""

import yaml, rospy
import sys
import os
import time
import rospkg
import subprocess
import rospy
import rostopic
import rospy.core
import numpy as np
from argparse import ArgumentParser
from std_msgs.msg import Bool


def read_in_user_data():
    dir = rospkg.RosPack().get_path('arena_bringup')

    # reading in user data
    parser = ArgumentParser()
    parser.add_argument("--yaml_path", action="store", dest="yaml_path", default=f"{dir}/launch/launch_configs/config2.yaml",
                        help="Path to your specified launch configs, previously specified",
                        required=False)  # TODO CHANGE TO TRUE
    args = parser.parse_args()

    with open(args.yaml_path, 'r') as stream:
        data = yaml.safe_load(stream)

    return data


def additional_simulation_settings(data: dict):
    gui = ''
    if not data['Visualization']:
        gui = 'use_rviz:=false headless:=true gui:=false'
    return gui


def get_world_name_from_scenario_name(path: str):
    world_dir = rospkg.RosPack().get_path('simulator_setup') + \
        '/worlds'
    worlds = os.listdir(world_dir)
    for world in worlds:
        if world in path:
            return world


def start_simulation(planer, model, scenario, settings):
    world_name = get_world_name_from_scenario_name(scenario)
    launch_command = f"roslaunch arena_bringup start_arena_gazebo.launch local_planner:={planer} model:={model} scenario_file:={scenario} task_mode:=scenario world:={world_name} {settings}"
    subprocess.Popen(launch_command, shell=True)


def wait_until_finish():
    """Waiting for the task-generator to publish, that he is finished with running the scenario
    """
    
    while True:
        pass

def terminate_simulation():

    # wait until ros core is shut down
    while not rospy.is_shutdown():
        time.sleep(1)
    time.sleep(3)
    
    # terminate rosnodes
    subprocess.Popen("rosnode kill --all", shell=True)

    # terminate gazebo
    subprocess.Popen(
        "killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient",
        shell=True,
    )


    # wait until ros core is shut down
    while not rospy.is_shutdown():
        time.sleep(1)
    time.sleep(1)



if __name__ == '__main__':

    data = read_in_user_data()
    settings = additional_simulation_settings(data)

    # specify launch files
    for planer in data['local_planner']:
        for model in data['model']:
            # Note to avoid some combinations: if planer == 'teb' and model == 'jackal': break
            for scenario in data['scenarios']:
                start_simulation(planer, model, scenario, settings)
                wait_until_finish()
                terminate_simulation()
    sys.exit(0)
