cmake_minimum_required(VERSION 2.8.3)
project(rovi2_demo_manager)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
