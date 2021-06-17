# Copyright 2021 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_mcl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(EXTRA_FLAGS -Wno-format-nonliteral -Wno-noexcept) # required for message_filter.h

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  ruvu_lint
  tf2_geometry_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system iostreams)

find_package(GRAPHICSMAGICKCPP REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/filter.cpp
  src/map.cpp
  src/motion_models/differential_motion_model.cpp
  src/motion_models/motion_model.cpp
  src/node.cpp
  src/particle_filter.cpp
  src/resamplers/low_variance.cpp
  src/resamplers/resampler.cpp
  src/rng.cpp
  src/sensor_models/beam_model.cpp
  src/sensor_models/laser.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_warnings(${PROJECT_NAME} ${EXTRA_FLAGS})

add_executable(node
  src/main.cpp
)
target_link_libraries(node
  ${PROJECT_NAME}
)
target_compile_warnings(node ${EXTRA_FLAGS})

add_executable(offline
  src/offline.cpp
)
target_link_libraries(offline
  ${PROJECT_NAME}
)
target_compile_warnings(offline ${EXTRA_FLAGS})

#############
## Install ##
#############

install(TARGETS node offline
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

#############
## Testing ##
#############

## Linting

# set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-build/include_what_you_use,-whitespace/braces,-runtime/references")
set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces")
ruvu_lint_add_test()

## Manual tests

if(CATKIN_ENABLE_TESTING)
  add_executable(plot_motion_models
    test/plot_motion_models.cpp
  )
  target_include_directories(plot_motion_models
    SYSTEM PRIVATE
      ${Boost_INCLUDE_DIRS}
  )
  target_link_libraries(plot_motion_models
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
  )

  add_executable(plot_resampling
    test/plot_resampling.cpp
  )
  target_include_directories(plot_resampling
    SYSTEM PRIVATE
      ${Boost_INCLUDE_DIRS}
  )
  target_link_libraries(plot_resampling
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
  )

  add_executable(render_scan
    test/render_scan.cpp
  )
  target_include_directories(render_scan
    SYSTEM PRIVATE
      ${Boost_INCLUDE_DIRS}
      ${GRAPHICSMAGICKCPP_INCLUDE_DIRS}
  )
  target_link_libraries(render_scan
    ${GRAPHICSMAGICKCPP_LIBRARIES}
    ${PROJECT_NAME}
  )
endif()

## Unit tests

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_differential_motion_model test/test_differential_motion_model.cpp)
  target_link_libraries(test_differential_motion_model ${PROJECT_NAME})

  catkin_add_gtest(test_map test/test_map.cpp)
  target_link_libraries(test_map ${PROJECT_NAME})
endif()
