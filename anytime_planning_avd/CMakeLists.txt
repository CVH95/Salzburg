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
  moveit_core
  moveit_commander
  moveit_msgs
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_move_group
  moveit_planners_ompl
  moveit_ros_visualization
  moveit_fake_controller_manager
  moveit_simple_controller_manager
  moveit_visual_tools
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES anytime_planning
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Libraries
add_library(anytime_planning src/anytime_planning.cpp)
target_link_libraries(anytime_planning ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

# Nodes
add_executable(random_ball_motion_node nodes/random_ball_motion_node.cpp)
target_link_libraries(random_ball_motion_node ${catkin_LIBRARIES} )

add_executable(anytime_planning_node nodes/anytime_planning_node.cpp)
target_link_libraries(anytime_planning_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} anytime_planning)

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