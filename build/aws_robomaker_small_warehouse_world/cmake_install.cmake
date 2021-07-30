# Install script for directory: /home/elias/catkin_ws/src/arena-rosnav-3D/src/aws-robomaker-small-warehouse-world

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/elias/catkin_ws/src/arena-rosnav-3D/install/aws_robomaker_small_warehouse_world")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/elias/catkin_ws/src/arena-rosnav-3D/build/aws_robomaker_small_warehouse_world/catkin_generated/installspace/aws_robomaker_small_warehouse_world.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world/cmake" TYPE FILE FILES
    "/home/elias/catkin_ws/src/arena-rosnav-3D/build/aws_robomaker_small_warehouse_world/catkin_generated/installspace/aws_robomaker_small_warehouse_worldConfig.cmake"
    "/home/elias/catkin_ws/src/arena-rosnav-3D/build/aws_robomaker_small_warehouse_world/catkin_generated/installspace/aws_robomaker_small_warehouse_worldConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE FILE FILES "/home/elias/catkin_ws/src/arena-rosnav-3D/src/aws-robomaker-small-warehouse-world/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/aws_robomaker_small_warehouse_world" TYPE DIRECTORY FILES
    "/home/elias/catkin_ws/src/arena-rosnav-3D/src/aws-robomaker-small-warehouse-world/launch"
    "/home/elias/catkin_ws/src/arena-rosnav-3D/src/aws-robomaker-small-warehouse-world/models"
    "/home/elias/catkin_ws/src/arena-rosnav-3D/src/aws-robomaker-small-warehouse-world/worlds"
    "/home/elias/catkin_ws/src/arena-rosnav-3D/src/aws-robomaker-small-warehouse-world/maps"
    "/home/elias/catkin_ws/src/arena-rosnav-3D/src/aws-robomaker-small-warehouse-world/rviz"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/elias/catkin_ws/src/arena-rosnav-3D/build/aws_robomaker_small_warehouse_world/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/elias/catkin_ws/src/arena-rosnav-3D/build/aws_robomaker_small_warehouse_world/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
