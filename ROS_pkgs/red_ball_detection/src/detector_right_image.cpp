// Obstacle Detection Shared Library

// Implemented for Robotics & Computer Vision 2 course 2018, SDU Robotics
// Carlos, Sergi, Richárd & Mathesh

// Object used as obstacle: Red Ball.

// ObstacleDetection.h

// ROS 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// OpenCV
#include <opencv2/opencv>
// General
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <numeric>

# define M_PI           3.14159265358979323846

using namespace std;
using namespace cv;


