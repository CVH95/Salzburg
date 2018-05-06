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


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "collision_detector");
	ros::NodeHandle nh;

	// Files
	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";
	const string filename = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/tests/plan.txt";

	const string scene = "/red_ball_detection/triangulated_ball_location";
	const string stat = "/planner/collisions";

	ros::Publisher booleanPub = nh.advertise<std_msgs::Bool>(stat, 1);

	// Subscribe to scene topic 
	

	AnytimePlanning plan;
	plan.Load_WorkCell(wcFile, deviceName);
	QPath exp_path = plan.read_path(filename);

	std_msgs::Bool msg;
	bool inCollision = plan.invalidate_nodes(exp_path, nh, scene);

	if( inCollision == true )
	{
	
		msg.data = true;
		// Send collision status to anytime_planner to stop the current path and do replanning
		booleanPub.publish(msg);
		// Send current Q of the robot?

	}// if

	else 
	{
		msg.data = false;
		booleanPub.publish(msg);	
	
	}// else

	ros::Duration(1).sleep();
	ros::spin();

	return 0;
}
