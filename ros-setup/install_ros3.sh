#!/usr/bin/env bash

workon rosnav
cd $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/env
pip install -r requirements_cadrl.txt
cd $HOME/catkin_ws/src/forks/stable-baselines3/
pip install -e .
cd $HOME/catkin_ws/src
git clone https://github.com/ros/genmsg.git -b kinetic-devel
git clone https://github.com/ros/dynamic_reconfigure.git -b melodic-devel
git clone https://github.com/ros/ros.git -b melodic-devel
git clone https://github.com/ros/genpy.git
git clone https://github.com/jsk-ros-pkg/geneus.git
git clone https://github.com/ros/gencpp.git -b kinetic-devel
git clone https://github.com/RethinkRobotics-opensource/gennodejs.git -b kinetic-devel
git clone https://github.com/ros/genlisp.git -b kinetic-devel
pip install filelock
cd $HOME/catkin_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
