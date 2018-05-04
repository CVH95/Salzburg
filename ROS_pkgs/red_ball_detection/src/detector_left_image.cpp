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
#include <geometry_msgs/PointStamped.h>
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
  	ros::init(argc, argv, "Ball_detector_left_image");

  	const string sub_topic_name = "/camera/left/image_raw";
	const string pub_topic_name = "/red_ball_detection/left_image_coordinates";
	//const string pub_display = "/red_ball_detection/hsv_left_image";

	// Use a while loop to control the refresh rate of the camera
	//while(ros::ok)
	//{	

		// Call detector
		ObstacleDetection LR(sub_topic_name, pub_topic_name);//, pub_display);
	
		// Update stereo info each 0.5 seconds
	//	ros::Duration(1.5).sleep();		
  	//	ros::spinOnce();
	
	//}// while

	ros::spin();

  	return 0;

} // main
