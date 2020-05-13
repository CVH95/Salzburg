cmake_minimum_required(VERSION 2.8.3)
project(anytime_planning_avd)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  roslib
  rovi2_msgs
  gazebo_msgs
  geometry_msgs
  tf2_ros
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES anytime_planning
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Libraries
add_library(anytime_planning src/anytime_planning.cpp)
target_link_libraries(anytime_planning ${catkin_LIBRARIES})

# Nodes
add_executable(random_ball_motion_node nodes/random_ball_motion_node.cpp)
target_link_libraries(random_ball_motion_node ${catkin_LIBRARIES} )

add_executable(anytime_planning_node nodes/anytime_planning_node.cpp)
target_link_libraries(anytime_planning_node ${catkin_LIBRARIES} anytime_planning)

install(TARGETS anytime_planning random_ball_motion_node anytime_planning_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)