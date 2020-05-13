cmake_minimum_required(VERSION 2.8.3)
project(perception_avd)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  roslib
  cv_bridge
  sensor_msgs
  rovi2_msgs
  geometry_msgs
  tf2_ros
  eigen_conversions
  message_filters
  kalman_tracking_3d
  pcl_conversions
)

find_package(OpenCV 3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES red_ball_detection stereopsis kinect_detection
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Libraries
add_library(common_perception src/common_perception.cpp)
target_link_libraries(common_perception ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(red_ball_detection src/red_ball_detection.cpp)
target_link_libraries(red_ball_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} common_perception)

add_library(stereopsis src/stereopsis.cpp)
target_link_libraries(stereopsis ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} common_perception)

add_library(kinect_detection src/kinect_detection.cpp)
target_link_libraries(kinect_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} common_perception)

# Nodes
add_executable(red_ball_detection_node nodes/red_ball_detection_node.cpp)
target_link_libraries(red_ball_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} common_perception red_ball_detection)

add_executable(stereopsis_node nodes/stereopsis_node.cpp)
target_link_libraries(stereopsis_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} common_perception stereopsis)

add_executable(monocular_node nodes/monocular_node.cpp)
target_link_libraries(monocular_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} common_perception stereopsis)

add_executable(kinect_detection_node nodes/kinect_detection_node.cpp)
target_link_libraries(kinect_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} kinect_detection)

install(TARGETS common_perception red_ball_detection stereopsis kinect_detection red_ball_detection_node stereopsis_node monocular_node kinect_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)