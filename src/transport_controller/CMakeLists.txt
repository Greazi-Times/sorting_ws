cmake_minimum_required(VERSION 3.0.2)
project(transport_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/Test_program_transportband.py
  src/transportband_handler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
