cmake_minimum_required(VERSION 3.0.2)
project(my_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  message_generation
)

## Declare Action file
add_action_files(
  DIRECTORY action
  FILES MoveToPose.action
)

## Generate messages and action support code
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    actionlib
    actionlib_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/run_pick.py
  src/robot_handler.py
  src/transfer_util.py
  src/robot_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


