// ROS 
#include <ros/ros.h>
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

using namespace std;
using namespace cv;

void chatterCallback(const red_ball_detection::ballCentrum msg)
{
	cout << endl;
	cout << "Ball coordinates:" << endl;
	cout << msg << endl;
	cout << endl;

} //chatterCallback() 


//----------------------------------------------------------------------------------------


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "Scene_3D_triangulation");
	ros::NodeHandle nh;

	cout << "LEFT IMAGE" << endl;
	ros::Subscriber subs_left = nh.subscribe<red_ball_detection::ballCentrum>("/red_ball_detection/left_image_coordinates", 1, chatterCallback);
	
	cout << "RIGHT IMAGE" << endl;
	ros::Subscriber subs_right = nh.subscribe<red_ball_detection::ballCentrum>("/red_ball_detection/right_image_coordinates", 1, chatterCallback);

	ros::spin();
	return 0;

}// main()
