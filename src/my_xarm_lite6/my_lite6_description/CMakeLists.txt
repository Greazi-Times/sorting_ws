cmake_minimum_required(VERSION 2.8.3)
project(my_lite6_description)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
endif()

install(DIRECTORY cfg config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
