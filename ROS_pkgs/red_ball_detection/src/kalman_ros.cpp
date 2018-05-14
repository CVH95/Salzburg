//#pragma once

// Existing code goes here


// ROS 
#include <ros/ros.h>

// OpenCV
#include <opencv2/opencv.hpp>
// Libs
#include <ObstacleDetection.h>
#include <kalman_3d.h>
// General
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <numeric>

using namespace std;
using namespace cv;

// ROVI2 - Object Avoidance 
//Collision Detector


//General


#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>

// Libs

// ROS
#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
// Libs

#include <Stereopsis.h>
#include <kalman_3d.h>


using namespace std;





// ----------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "kalman_filter");
	ros::NodeHandle nh;

	const string pub_name = "/red_ball_detection/kalman_ball_location";
	const string sub_name = "/red_ball_detection/triangulated_ball_location";

	KALMAN kf(nh, pub_name);
	//kf.accelerationKF();
	kf.sub_kf(nh, sub_name);
	//kf.pub_kf(nh, pub_name);
	
	ros::spin();

	return 0;
}
