execute_process(COMMAND "/home/elias/catkin_ws/src/arena-rosnav-3D/build/task_generator_gazebo/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/elias/catkin_ws/src/arena-rosnav-3D/build/task_generator_gazebo/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
