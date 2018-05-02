// ROS 
#include <ros/ros.h>
#include <red_ball_detection/ballCentrum.h> // Created msg format
#include <red_ball_detection/ballToRobotBase.h> // Created msg format
// OpenCV
#include <opencv2/opencv.hpp>
// Libs
#include <ObstacleDetection.h>
#include <Stereopsis.h>
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

/*void chatterCallback(const red_ball_detection::ballCentrum msg)
{
	cout << endl;
	cout << "Ball coordinates:" << endl;
	cout << msg << endl;
	cout << endl;

} //chatterCallback() */


//----------------------------------------------------------------------------------------


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "Scene_3D_triangulation");
	ros::NodeHandle nh;

	cout << "Scene_3D_triangulation" << endl << endl;

	const string left_sub_name = "/red_ball_detection/left_image_coordinates";
	const string right_sub_name = "/red_ball_detection/right_image_coordinates";
	const string pub_name = "/red_ball_detection/triangulated_ball_location";
	const string file = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/ROS_pkgs/red_ball_detection/calibration/BumbleBee_from_robot.txt";

	/*cout << "LEFT IMAGE" << endl;
	ros::Subscriber subs_left = nh.subscribe<red_ball_detection::ballCentrum>("/red_ball_detection/left_image_coordinates", 1, chatterCallback);
	
	cout << "RIGHT IMAGE" << endl;
	ros::Subscriber subs_right = nh.subscribe<red_ball_detection::ballCentrum>("/red_ball_detection/right_image_coordinates", 1, chatterCallback);*/

	// Calling Stereopsis Object constructor
	Stereopsis triangulation(nh, pub_name);
	StereoPair stPair;

	// Build projection matrices
	bool read = triangulation.readStereoCameraFile(file, stPair);
	cout << "File reading status: " << read << endl;

	Mat proj_l = triangulation.constructProjectionMat(stPair.cam1);
	Mat proj_r = triangulation.constructProjectionMat(stPair.cam2);

	// Obtain image coordinates
	vector<float> px_c = triangulation.group_coordinates(nh, left_sub_name, right_sub_name);
	cout << "Subscription coordinates:  " << px_c[0] << ", " << px_c[0] << ", " << px_c[0] << ", " << px_c[0] << endl;

	// Execute triangulation
	vector<float> t = triangulation.calculate_3D_location(px_c, proj_l, proj_r);
	
	// Clear vector for new entries
	// px_c.clear();
	
	// Broadcast triangulated coordinates into the network
	triangulation.broadcast_3D_location(t);

	ros::spin();
	return 0;

}// main()
