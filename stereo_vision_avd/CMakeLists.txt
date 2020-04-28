cmake_minimum_required(VERSION 2.8.3)
project(stereo_vision_avd)

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
)

find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES red_ball_detection stereopsis
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Libraries
add_library(red_ball_detection src/red_ball_detection.cpp)
target_link_libraries(red_ball_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(stereopsis src/stereopsis.cpp)
target_link_libraries(stereopsis ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Nodes
add_executable(red_ball_detection_node nodes/red_ball_detection_node.cpp)
target_link_libraries(red_ball_detection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} red_ball_detection)

add_executable(stereopsis_node nodes/stereopsis_node.cpp)
target_link_libraries(stereopsis_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stereopsis)

install(TARGETS red_ball_detection stereopsis red_ball_detection_node stereopsis_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Launch test
# if(CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS
#     roslaunch
#   )
#   roslaunch_add_file_check(launch)
# endif()