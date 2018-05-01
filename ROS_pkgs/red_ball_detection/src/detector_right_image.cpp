// Obstacle Detection
// Implemented for Robotics & Computer Vision 2 course 2018, SDU Robotics
// Carlos, Sergi, Richárd & Mathesh

// Object used as obstacle: Red Ball.



// ROS 
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <red_ball_detection/ballCentrum.h> // Created msg format
// OpenCV
#include <opencv2/opencv.hpp>
// Libs
#include <ObstacleDetection.h>
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

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "Ball_detector_right_image");

  	const string sub_topic_name = "/camera/right/image_raw";
	const string pub_topic_name = "/red_ball_detection/right_image_coordinates";
	const string pub_display = "/red_ball_detection/hsv_right_image";

	ObstacleDetection DR(sub_topic_name, pub_topic_name, pub_display);
	
  	ros::spin();
  	return 0;

} // main
