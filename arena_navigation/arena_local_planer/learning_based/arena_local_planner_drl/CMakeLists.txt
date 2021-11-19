cmake_minimum_required(VERSION 3.0.2)
project(arena_local_planner_drl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()



catkin_package(

)

include_directories(
# include
  scripts
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
#    scripts/env/flatland_gym_env.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
