cmake_minimum_required(VERSION 3.0.2)
project(transport_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller_node src/controller_node.cpp)

target_link_libraries(controller_node
  ${catkin_LIBRARIES}
)
