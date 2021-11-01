cmake_minimum_required(VERSION 3.0.2)
project(simulator_setup)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  gazebo_msgs
  pedsim_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED )
link_directories(${GAZEBO_LIBRARY_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS gazebo_ros roscpp  geometry_msgs 
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


#add_library(actor_collisions_plugin SHARED /obstacles/utils/ActorCollisionsPlugin.cpp)
#target_link_libraries(actor_collisions_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(actor_collisions_plugin ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############