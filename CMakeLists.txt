cmake_minimum_required(VERSION 3.0.2)
project(ros_slam_webui)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(DIRECTORY web launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
