cmake_minimum_required(VERSION 3.10.2)

project(depthai_ros_msgs VERSION 2.6.4)

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Already handled by find_package(catkin ...)
# set(catkin_FOUND 1)  <-- This is non-standard and unnecessary

add_definitions(-DUSING_ROS)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
)

######################################
## Declare ROS messages and services #
######################################

# Generate messages
add_message_files(
  FILES
  AutoFocusCtrl.msg
  HandLandmark.msg
  HandLandmarkArray.msg
  SpatialDetection.msg
  SpatialDetectionArray.msg
)

# Generate services
add_service_files(
  FILES
  TriggerNamed.srv
  NormalizedImageCrop.srv
  camera.srv
)

# Generate both messages and services
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  vision_msgs
)

#############################
## Catkin package settings ##
#############################

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
    vision_msgs
)
