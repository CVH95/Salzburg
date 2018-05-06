cmake_minimum_required(VERSION 2.8.3)
project(robot_plugin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  caros_common_msgs
  caros_common_robwork
  caros_control_msgs
  caros_control
  caros_common
#
)


#add_service_files(
#  FILES  
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)
SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWSTUDIO_ROOT "$ENV{RWS_ROOT}")

  MESSAGE(STATUS ${RW_ROOT})
  MESSAGE(STATUS ${catkin_INCLUDE_DIRS})



#Include default settings for constructing a robwork dependent project
FIND_PACKAGE(
  RobWork
  QUIET
  HINTS
    "${RW_ROOT}")
#The following is a workaround for the old version of RobWork. Remove once no longer needed (everyone switched to a RobWork version newer than April 2012)
IF(NOT ROBWORK_FOUND)
  MESSAGE(STATUS "Could not find RobWork with the new method. Trying the old one.")
  INCLUDE(${RW_ROOT}/build/FindRobWork.cmake)
ENDIF(NOT ROBWORK_FOUND)

#Include default settings for constructing a robworkst dependent project
FIND_PACKAGE(
  RobWorkStudio
  QUIET
  HINTS
    "${RWSTUDIO_ROOT}")
#The following is a workaround for the old version of RobWorkSt. Remove once no longer needed (everyone switched to a RobWork version newer than April 2012)
IF(NOT ROBWORKSTUDIO_FOUND)
  MESSAGE(STATUS "Could not find RobWorkStudio with the new method. Trying the old one.")
  INCLUDE(${RWSTUDIO_ROOT}/build/FindRobWorkStudio.cmake)
ENDIF(NOT ROBWORKSTUDIO_FOUND)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${catkin_LIBRARIES})


FIND_PACKAGE(Qt5Widgets REQUIRED)

# if we want to use ui files add them here
SET(UIS_FILES src/RobotPlugin.ui )

QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})

QT5_WRAP_CPP(MocSrcFiles src/RobotPlugin.hpp src/qtros.h
OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
)
# Rcc the files:

#QT5_WRAP_CPP(sources $

#{moc-sources}
#OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)

#QT5_ADD_RESOURCES(RccSrcFiles resources.qrc)
set(SrcFiles
	src/RobotPlugin.cpp
        src/qtros.cpp
	${UIS_OUT_H}
)


set(ROS_BUILD_STATIC_LIBS false)
set(ROS_BUILD_SHARED_LIBS true)


### Linkig AnytimePlanning to the Plugin

# include directories for shared libraries
include_directories( /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/ROS_pkgs/planner/include )

# Find Library
find_package(
  AnytimePlanning
)


#target_link_libraries(RWSPlugin ${catkin_LIBRARIES})
add_library(robot_plugin ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
target_link_libraries(robot_plugin ${catkin_LIBRARIES})
add_dependencies(robot_plugin ${catkin_EXPORTED_TARGETS}})
