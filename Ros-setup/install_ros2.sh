#!/usr/bin/env bash

mkvirtualenv --python=python3.6 rosnav
workon rosnav
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip install pyyaml catkin_pkg netifaces pathlib
pip install stable-baselines3
cd $HOME
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/ignc-research/arena-rosnav
cd arena-rosnav && rosws update
source $HOME/.zshrc
cd $HOME/catkin_ws/src/
git clone https://github.com/eliastreis/arena-rosnav-3D.git
cd ../.. 
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
cd $HOME/catkin_ws/src/arena-rosnav
./geometry2_install.sh
cd $HOME/geometry2_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

