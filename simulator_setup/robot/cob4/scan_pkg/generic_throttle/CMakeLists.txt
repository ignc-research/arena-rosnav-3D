cmake_minimum_required(VERSION 3.0.2)
project(generic_throttle)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS scripts/${PROJECT_NAME}_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
