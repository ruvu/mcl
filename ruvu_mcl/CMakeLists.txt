# Copyright 2021 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_mcl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
  set(EXTRA_FLAGS -Wno-format-nonliteral)
else()
  set(EXTRA_FLAGS -Wno-format-nonliteral -Wno-noexcept -Wstrict-overflow=2) # required for message_filter.h
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
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

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/AMCL.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/config.cpp
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
  src/sensor_models/likelihood_field_model.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
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

# 0.12.0
if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  if (roslint_VERSION VERSION_GREATER 0.12.0)
    set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces")
  else()
    set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces,-build/c++11")
  endif()
endif()

set(ROSLINT_PYTHON_OPTS "--max-line-length=305")
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

  catkin_add_gtest(test_occupancy_map test/test_occupancy_map.cpp)
  target_link_libraries(test_occupancy_map ${PROJECT_NAME})

  catkin_add_gtest(test_distance_map test/test_distance_map.cpp)
  target_link_libraries(test_distance_map ${PROJECT_NAME})
endif()
