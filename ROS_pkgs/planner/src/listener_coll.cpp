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
#include <std_msgs/Bool.h>



using namespace std;


void callback(const std_msgs::Bool::ConstPtr &msg)
{

	cout << "Found Collisions?   " << msg->data << endl;

}//callback()


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "collision_detector_test_listener");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe<std_msgs::Bool>("/planner/collisions", 1, callback);

	ros::spin();
	return 0;

}// main()
