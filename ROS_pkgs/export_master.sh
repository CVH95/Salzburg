#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/charlie/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.100.52:11311/
export ROS_IP=192.168.100.25
echo "Connected to ROVI2 WorkCell Station 2"
echo "ROS_MASTER_URI set to 192.168.100.52:11311"
echo "ROS_IP set to 192.168.100.25"
