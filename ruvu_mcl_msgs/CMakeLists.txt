# Copyright 2021 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_mcl_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
    LandmarkEntry.msg
    LandmarkList.msg
)

add_service_files(
  FILES
  AddLandmark.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
