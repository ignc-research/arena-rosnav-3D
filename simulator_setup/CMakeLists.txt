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
  controller_interface
  realtime_tools
  tf
  urdf
  nav_msgs
)

set(mecanum_drive_controller_sources
  robot/ridgeback/control/src/mecanum_drive_controller.cpp
  robot/ridgeback/control/src/odometry.cpp
  robot/ridgeback/control/src/speed_limiter.cpp
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED )
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

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
  INCLUDE_DIRS include
  LIBRARIES mecanum_drive_controller
  CATKIN_DEPENDS controller_interface realtime_tools gazebo_ros roscpp  geometry_msgs  std_msgs nav_msgs tf message_runtime
)


###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(ridgeback_ros_force_based_move robot/ridgeback/control/src/ridgeback_ros_force_based_move.cpp)
target_link_libraries(ridgeback_ros_force_based_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


add_library(mecanum_drive_controller ${mecanum_drive_controller_headers} ${mecanum_drive_controller_sources})
target_link_libraries(mecanum_drive_controller ${catkin_LIBRARIES})


#add_library(actor_collisions_plugin SHARED /obstacles/utils/ActorCollisionsPlugin.cpp)
#target_link_libraries(actor_collisions_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(actor_collisions_plugin ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(DIRECTORY include/ridgeback_gazebo_plugins/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  ridgeback_ros_force_based_move
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(TARGETS mecanum_drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES robot/ridgeback/control/mecanum_drive_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)