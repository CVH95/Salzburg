// ROS 
#include <ros/ros.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
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
#include <boost/bind.hpp>


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


void image_sync_callback(
	ros::NodeHandle &nh,
	Stereopsis &triangulation,
	Mat &proj_l,
	Mat &proj_r,
	const red_ball_detection::ballCentrum::ConstPtr & left_subs,
 	const red_ball_detection::ballCentrum::ConstPtr & right_subs)
{
  	vector<float> px_c = triangulation.group_coordinates(*left_subs, *right_subs);
	
	// Execute triangulation
	vector<float> t = triangulation.calculate_3D_location(px_c, proj_l, proj_r);
	cout << "Triangulated coordinates X, Y, Z:  " << t[0] << ", " << t[1] << ", " << t[2] << endl;

	// Broadcast triangulated coordinates into the network
	triangulation.broadcast_3D_location(t);
}


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "Scene_3D_triangulation");
	ros::NodeHandle nh;
	
	const string left_sub_name = "/red_ball_detection/left_image_coordinates";
	const string right_sub_name = "/red_ball_detection/right_image_coordinates";
	const string pub_name = "/red_ball_detection/triangulated_ball_location";
	const string file = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/ROS_pkgs/red_ball_detection/calibration/BumbleBee_from_robot.txt";

	// Calling Stereopsis Object constructor
	Stereopsis triangulation(nh, pub_name);
	StereoPair stPair;

	cout << "Scene_3D_triangulation" << endl << endl;

	// Build projection matrices
	bool read = triangulation.readStereoCameraFile(file, stPair);
	cout << "File reading status: " << read << endl;

	Mat proj_l = triangulation.constructProjectionMat(stPair.cam1);
	Mat proj_r = triangulation.constructProjectionMat(stPair.cam2);

	//Subscribtion and syncronization
	message_filters::Subscriber<red_ball_detection::ballCentrum> left_subs(nh, left_sub_name, 1);
    message_filters::Subscriber<red_ball_detection::ballCentrum> right_subs(nh,right_sub_name, 1);
    message_filters::TimeSynchronizer<red_ball_detection::ballCentrum, red_ball_detection::ballCentrum> sync(left_subs, right_subs, 10);
    sync.registerCallback(boost::bind(
		&image_sync_callback,
		boost::ref(nh), 
		boost::ref(triangulation), 
		boost::ref(proj_l),
		boost::ref(proj_r),
		_1, 
		_2));

	ros::spin();
	return 0;

}// main()
