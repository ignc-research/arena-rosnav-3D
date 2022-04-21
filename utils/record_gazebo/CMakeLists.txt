cmake_minimum_required(VERSION 3.0.2)
project(record_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/record_gazebo_node.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)