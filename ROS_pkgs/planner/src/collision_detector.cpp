// ROVI2 - Object Avoidance 
//Collision Detector


//General
#include <iostream>
#include <fstream> 
#include <vector>
#include <math.h> 
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <string>
// Libs
#include <AnytimePlanning.h>
//#include <URRobot.h>
// RobWork
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/CubicSplineFactory.hpp>
// ROS
#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>


using namespace std;


// ----------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "collision_detector");
	ros::NodeHandle nh;

	// Files
	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";
	//const string filename = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/tests/plan.txt";

	//const string scene = "/red_ball_detection/triangulated_ball_location";
	const string stat = "/planner/collisions";

	AnytimePlanning plan;
	
	plan.booleanPub = nh.advertise<std_msgs::Bool>(stat, 1);

	plan.Load_WorkCell(wcFile, deviceName);
	
	plan.find_obstacles(nh);
	//ros::Subscriber subs = nh.subscribe( "/red_ball_detection/triangulated_ball_location", 1, &ball_location_callback);

	//ros::Duration(1).sleep();*/
	//ros::spin();

	return 0;
}
