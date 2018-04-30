cmake_minimum_required(VERSION 2.8.3)
project(red_ball_detection)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  roslibs
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
)


# Generating new messages
add_message_files(
  FILES 
  ballCentrum.msg  
  ballToRobotBase.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# catkin shit
catkin_package( CATKIN_DEPENDS message_runtime )

#includes 
# Do not forget to declare the /include directory if a shared library is built
include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

# Shared Libraries
#add_librarby (<name> SHARED src/<source_code>.cpp)

# Build Detector Node
add_executable(stereo_detector src/detector.cpp)
target_link_libraries( stereo_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ) 

# Build Stereo Triangulation Node
add_executable(stereo_triangulation src/triangulation.cpp)
target_link_libraries( stereo_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ) 
