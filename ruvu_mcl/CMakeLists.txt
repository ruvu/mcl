cmake_minimum_required(VERSION 3.0.2)
project(ruvu_mcl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  pf
  roscpp
  tf2_ros
  tf2_geometry_msgs
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

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/map.cpp
  src/motion_models/differential_motion_model.cpp
  src/node.cpp
  src/particle_filter.cpp
  src/sensor_models/beam_model.cpp
  src/sensor_models/laser.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(node
  src/main.cpp
)
target_link_libraries(node
  ${PROJECT_NAME}
)

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

#############
## Testing ##
#############

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

  message("GRAPHICSMAGICKCPP_LIBRARIES: ${GRAPHICSMAGICKCPP_LIBRARIES}")
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

catkin_add_gtest(test_differential_motion_model test/test_differential_motion_model.cpp)
if(TARGET test_differential_motion_model)
  target_link_libraries(test_differential_motion_model ${PROJECT_NAME})
endif()

catkin_add_gtest(test_map test/test_map.cpp)
if(TARGET test_map)
  target_link_libraries(test_map ${PROJECT_NAME})
endif()
