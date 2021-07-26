#!/bin/bash

sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
cd ~/catkin_ws/src
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b melodic-devel
cd ..
catkin_make

# to install the navigation/slam pkg MRP2
#cd ~/catkin_ws/src && git clone https://github.com/ros-visualization/interactive_marker_twist_server.git \
#sudo apt-get install ros-melodic-roslint \
#git clone https://github.com/ros-controls/ros_controllers.git -b melodic-devel \
#git clone https://github.com/ros-drivers/four_wheel_steering_msgs.git \
#git clone https://github.com/ros-controls/urdf_geometry_parser.git -b kinetic-devel \
#git clone https://github.com/cra-ros-pkg/robot_localization.git -b melodic-devel

#cd.. && catkin_make
#cd ~/catkin_ws/src && git clone https://github.com/milvusrobotics/mrp2_common.git
#cd .. && catkin_make

#cd ~/Desktop/ros_ws/src && git clone https://github.com/milvusrobotics/mrp2_simulator.git -b melodic-devel # hier ev. ~/catkin_ws/src
#cd .. && catkin_make

#cd ~/Desktop/ros_ws/src && git clone https://github.com/ros-teleop/twist_mux.git -b melodic-devel # hier ev. ~/catkin_ws/src
#cd .. && catkin_make

# to install SLAM ros_autonomous_slam
cd ~/Desktop/ros_ws/src && git clone https://github.com/fazildgr8/ros_autonomous_slam.git
cd .. && catkin_make
