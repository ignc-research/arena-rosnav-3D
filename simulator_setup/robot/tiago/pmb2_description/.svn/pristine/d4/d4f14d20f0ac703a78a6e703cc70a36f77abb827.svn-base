cmake_minimum_required(VERSION 2.8.3)
project(pmb2_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config gazebo Media meshes robots urdf)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  foreach(laser_model false hokuyo sick-551 sick-561 sick-571)
      foreach(rgbd_sensors false true)
        add_rostest(test/test_pmb2.test ARGS laser_model:=${laser_model} rgbd_sensors:=${rgbd_sensors})
      endforeach(rgbd_sensors)
  endforeach(laser_model)
endif()
