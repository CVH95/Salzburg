cmake_minimum_required(VERSION 2.8.3)
project(planner)

## Add support for C++11, supported in ROS Kinetic and newer
set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  caros_common_msgs
  caros_control_msgs
  caros_common
  caros_control
  std_msgs
  caros_common_robwork
  geometry_msgs
  message_generation
  rosconsole
  roslint
)

###################################
##         RobWork setup         ##
###################################
# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Use RobWork
MESSAGE("-- RobWork root: " ${RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rovi2_rob
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Build Shared Libraries
add_library(AnytimePlanning SHARED src/AnytimePlanning.cpp)
add_library(URRobot SHARED src/URRobot.cpp)

# Build planner executable
add_executable(planner src/planner.cpp)
target_link_libraries(planner AnytimePlanning URRobot ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROS_LIBRARIES})
add_dependencies(planner caros_common_msgs caros_control_msgs caros_control)

# Build anytime_planner executable 
add_executable(anytime_planner src/anytime_planner.cpp)
target_link_libraries(anytime_planner AnytimePlanning URRobot ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROS_LIBRARIES})
add_dependencies(anytime_planner caros_common_msgs caros_control_msgs caros_control)

# Build anytime_planner executable 
add_executable(collision_detector src/collision_detector.cpp)
target_link_libraries(collision_detector AnytimePlanning URRobot ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROS_LIBRARIES})
#add_dependencies(collision_detector caros_common_msgs caros_control_msgs caros_control)

# Build ball coordinates test listener executable 
add_executable(triangulation_listener src/listener_ball.cpp)
target_link_libraries(triangulation_listener ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# Build collision boolean test listener executable
add_executable(collision_listener src/listener_coll.cpp)
target_link_libraries(collision_listener ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# Build test_collisions.cpp
add_executable(test_collision src/test_collision.cpp)
target_link_libraries(test_collision AnytimePlanning URRobot ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES} ${ROS_LIBRARIES})

MESSAGE(${PROJECT_NAME} " done!")
