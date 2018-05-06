#include <iostream>
#include <fstream> 
#include <vector>
#include <math.h> 
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <string>
#include <vector>
// ROS headers
#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>



using namespace std;


void callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{

	cout << "X = " << msg->point.x << ",  Y = " << msg->point.y << ",  Z = " << msg->point.z << endl;

}//callback()


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "stereo_test_listener");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("/red_ball_detection/triangulated_ball_location", 1, callback);

	ros::spin();
	return 0;

}// main()
