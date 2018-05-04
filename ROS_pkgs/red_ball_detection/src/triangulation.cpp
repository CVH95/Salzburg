// ROS 
#include <ros/ros.h>
#include <red_ball_detection/ballCentrum.h> // Created msg format
#include <red_ball_detection/ballToRobotBase.h> // Created msg format
#include <geometry_msgs/PointStamped.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
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


	// Calling Stereopsis Object constructor
	Stereopsis triangulation(nh, pub_name);

	// Build projection matrices
	bool read = triangulation.readStereoCameraFile(file);
	cout << "File reading status: " << read << endl;

	triangulation.getProjectionMat();
	
	// Call Subscription synchronizer 
	message_filters::Subscriber<geometry_msgs::PointStamped> image_left(nh, left_sub_name, 1);
	message_filters::Subscriber<geometry_msgs::PointStamped> image_right(nh, right_sub_name, 1);
	message_filters::TimeSynchronizer<geometry_msgs::PointStamped, geometry_msgs::PointStamped> sync(image_left, image_right, 10);

	// boost::bind requires an extra argument which is a pointer to the instance of that class.
	sync.registerCallback(boost::bind(&Stereopsis::synchronized_triangulation, triangulation, _1, _2));

	ros::spin();

	return 0;

}// main()
