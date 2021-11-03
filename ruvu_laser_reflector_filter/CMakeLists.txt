# Copyright 2021 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_laser_reflector_filter)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  ruvu_lint
  ruvu_mcl_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/ReflectorFilter.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
    ruvu_mcl_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(euclidean_clustering
  src/euclidean_clustering.cpp
)

add_executable(intensity_filter
  src/intensity_filter.cpp
)

target_link_libraries(intensity_filter
  euclidean_clustering
  ${catkin_LIBRARIES}
)

add_dependencies(intensity_filter ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/intensity_vs_distance
  scripts/sick_extended_data_extractor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    euclidean_clustering
    intensity_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces")
ruvu_lint_add_test()
