# Copyright 2021 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_mcl_rviz_plugins)

add_compile_options(-std=c++11)
set(EXTRA_FLAGS -Wno-deprecated-copy)
if(CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
  set(EXTRA_FLAGS ${EXTRA_FLAGS} -Wno-deprecated-register ) # required for OGRE
else()
  set(EXTRA_FLAGS ${EXTRA_FLAGS} -Wno-unused -Wno-switch-default -Wno-noexcept) # required for OGRE
endif()

find_package(catkin REQUIRED COMPONENTS
  ruvu_lint
  ruvu_mcl_msgs
  rviz
)

catkin_package(CATKIN_DEPENDS
    rviz
    ruvu_mcl_msgs)

###########
## Build ##
###########

# Inspired by http://wiki.ros.org/visualization_tutorials

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
  src/add_landmark_tool.cpp
)
target_compile_warnings(${PROJECT_NAME} ${EXTRA_FLAGS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY
  icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces")
ruvu_lint_add_test()
