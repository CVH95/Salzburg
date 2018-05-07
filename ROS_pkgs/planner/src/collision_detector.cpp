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

// Global variables
AnytimePlanning plan;
float x, y, z;
const string filename = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/tests/plan.txt";
ros::Publisher booleanPub;



// Callback function
void ball_location_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	x = msg->point.x;
	y = msg->point.y;
	z = msg->point.z;
	//cout << "Ball in position:   (" << x << ", " << y << ", " << z << ")" << endl;

	// // Create timer to get runtime 
	long countdown = 5000; // miliseconds
	rw::common::Timer timer = rw::common::Timer(countdown);
	timer.resetAndPause(); 

	QPath exp_path = plan.read_path(filename);

	timer.resetAndResume(); // Start timer

	//std_msgs::Bool msg;
	bool inCollision = plan.invalidate_nodes(exp_path, x, y, z);

	double time_past = timer.getTime();
	timer.resetAndPause();

	cout << "Path checked in " << time_past << " seconds." << endl;
	
	if( inCollision == true )
	{
		std_msgs::Bool msg;
		msg.data = true;
		// Send collision status to anytime_planner to stop the current path and do replanning
		booleanPub.publish(msg);
		// Send current Q of the robot?

	}// if

	else 
	{
		std_msgs::Bool msg;
		msg.data = false;
		booleanPub.publish(msg);	
	
	}// else


}// ball_location_callback



// ----------------------------------------------------------------------------------------------------------------------------------


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

	booleanPub = nh.advertise<std_msgs::Bool>(stat, 1);

	plan.Load_WorkCell(wcFile, deviceName);
	
	ros::Subscriber ball_subs = nh.subscribe<geometry_msgs::PointStamped>(scene, 1, &ball_location_callback);

	/* // Create timer to get runtime 
	long countdown = 5000; // miliseconds
	rw::common::Timer timer = rw::common::Timer(countdown);
	timer.resetAndPause(); 

	QPath exp_path = plan.read_path(filename);

	timer.resetAndResume(); // Start timer

	//std_msgs::Bool msg;
	bool inCollision = plan.invalidate_nodes(exp_path, x, y, z);

	double time_past = timer.getTime();
	timer.resetAndPause();

	cout << "Path checked in " << time_past << " seconds." << endl;
	
	if( inCollision == true )
	{
		std_msgs::Bool msg;
		msg.data = true;
		// Send collision status to anytime_planner to stop the current path and do replanning
		booleanPub.publish(msg);
		// Send current Q of the robot?

	}// if

	else 
	{
		std_msgs::Bool msg;
		msg.data = false;
		booleanPub.publish(msg);	
	
	}// else

	//ros::Duration(1).sleep();*/
	ros::spin();

	return 0;
}
