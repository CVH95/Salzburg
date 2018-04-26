// ROVI2 - Object avoidance: Planner node

// Q_start   --->   Q[6]{-1.04475, -1.78024, -2.24013, -0.781733, 1.61513, 0.0356047} (rad)
// Q_goal    --->   Q[6]{2.64138, -1.68337, -1.87675, -2.10958, 4.75969, 0.0352557} (rad)

#include <AnytimePlanning.h>

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
// RobWork headers
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/CubicSplineFactory.hpp>
// Caros and ROS headers
#include "ros/ros.h"
#include <caros/common.h>
#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>


using namespace std;

#define MAXTIME 1000.


void chatterCallback(const caros_control_msgs::RobotState msg)
{

	cout << "ROBOT STATE:" << endl;
	cout << msg << endl;

} //chatterCallback() 

//---------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	//rw::math::Math::seed();
	ros::init(argc, argv, "Online_Planner_Node");
	ros::NodeHandle nh;

	// LOADING WORKCELL 
	
	cout << endl;
	cout << "C++ SHARED LIBRARIES EXAMPLE" << endl; 
	cout << endl;

	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";

	AnytimePlanning plan;
	plan.Load_WorkCell(wcFile, deviceName);

	ros::Subscriber RS = nh.subscribe("/caros_universalrobot/caros_serial_device_service_interface/robot_state", 1, chatterCallback);
	
	/*cout << endl;
	cout << "Finished." << endl;
	cout << "Exiting." << endl;
	cout << endl;*/


	ros::spin(); // The program will be finished with 'ctrl+c' keyboard interrupt.
 	return 0;

} // main
