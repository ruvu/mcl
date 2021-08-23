# Copyright 2021 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_mcl_landmark_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  ruvu_mcl_msgs
  tf2_geometry_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp ruvu_mcl_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(landmark_server
  src/main.cpp
)

target_link_libraries(landmark_server
  ${catkin_LIBRARIES}
)
target_link_libraries(landmark_server $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>)

#############
## Install ##
#############

install(
  TARGETS
    landmark_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ruvu_mcl_landmark_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
