cmake_minimum_required(VERSION 2.8.3)
project(red_ball_detection)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  roslib
  std_msgs
  geometry_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
)

find_package(OpenCV 3 REQUIRED)


# Generating new messages
add_message_files(
  FILES 
  ballCentrum.msg  
  ballToRobotBase.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin shit
catkin_package( CATKIN_DEPENDS message_runtime )

#includes 
# Do not forget to declare the /include directory if a shared library is built
include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

# Shared Libraries
add_library(ObstacleDetection SHARED src/ObstacleDetection.cpp)
add_library(Stereopsis SHARED  src/Stereopsis.cpp)

# Build Detector (Right image) Node
add_executable(stereo_detector_destro src/detector_right_image.cpp)
target_link_libraries( stereo_detector_destro ObstacleDetection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ROS_LIBRARIES} )

# Build Detector (Left image) Node
add_executable(stereo_detector_sinistro src/detector_left_image.cpp)
target_link_libraries( stereo_detector_sinistro ObstacleDetection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ROS_LIBRARIES} ) 

# Build Stereo Triangulation Node
# add_executable(stereo_triangulation src/triangulation.cpp src/methods.hpp)
add_executable(stereo_triangulation src/triangulation.cpp )
target_link_libraries( stereo_triangulation ObstacleDetection Stereopsis ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ROS_LIBRARIES} ) 
#target_link_libraries( stereo_triangulation ObstacleDetection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ROS_LIBRARIES} ) 

# Install Launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
