#! /bin/bash

## 1. Set Environment Variables
export WORKON_HOME=/root/.python_env
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
export PYTHONPATH=/root/catkin_ws/src/arena-rosnav-3D:${PYTHONPATH}

source /usr/local/bin/virtualenvwrapper.sh
source /root/.bashrc
source /opt/ros/noetic/setup.bash
source $HOME/catkin_ws/devel/setup.bash

## 2. Activate Venv
workon rosnav 

## 3. Start simulation 
roslaunch arena_bringup start_arena_gazebo.launch \
local_planner:=$1 \
task_mode:=$2 \
world:=$3 \
model:=$4 \
actors:=$5 


