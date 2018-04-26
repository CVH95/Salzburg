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
#include <caros/ur_service_interface.h>


using namespace std;

#define MAXTIME 1000.



//---------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	//rw::math::Math::seed();
	ros::init(argc, argv, "planner");
	ros::NodeHandle nh;

	// LOADING WORKCELL & INITIAL SETTINGS
	
	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";

	AnytimePlanning plan;
	plan.Load_WorkCell(wcFile, deviceName);

	// Initial configuration defined		
	rw::math::Q from_deg(6, -59.86, -102.00, -128.35, -44.79, 92.54, 2.04); // deg
	rw::math::Q from = from_deg*(3.14159265359/180); // rad

	// Goal configuration defined
	rw::math::Q to_deg(6,151.34, -96.45, -107.53, -120.87, 272.71, 2.02); // deg
	rw::math::Q to = to_deg*(3.14159265359/180); // rad

	// Initial and final velocities for interpolating trajectory
	rw::math::Q dq_start(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	rw::math::Q dq_end(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	double epsilon = 0.5; //RRT's extend parameter

	// Subscribe to get information about the robot state (this is now implemented in a separate node named 'robot_state_monitoring'.
	// ros::Subscriber RS = nh.subscribe("/caros_universalrobot/caros_serial_device_service_interface/robot_state", 1, chatterCallback);
	

	// Set node to be client of "/caros_universalrobot/caros_serial_device_service_interface/move_servo_q".
	// In order to broadcast Q's belonging to the path, it is required that the client node sends Q request msgs to the server in charge of moving the robot. 
	ros::ServiceClient client = nh.serviceClient<caros_control_msgs::SerialDeviceServoQ>("/caros_universalrobot/caros_serial_device_service_interface/move_servo_q");
	caros_control_msgs::SerialDeviceServoQ srv;

	/*

	// PATH PLANNING

	// Calculate path using RRT* algorithm
	QPath raw_path = plan.get_path(epsilon, from, to);
	// Interpolate the obtained path to create a smooth trajectory
	QPath trajectory = plan.get_trajectory(raw_path, dq_start, dq_end);
	//Convert trajectory into ROS readable type
	vector<caros::caros_common_msgs::Q> trajROS = plan.convert_trajectory(trajectory);

	// SEND (NUDES) PATH
	int imax = trajROS.size();
	for (int i; i<imax; i++)
	{
		// This should be able to move the robot
		caros::caros_common_msgs::Q q;
		srv.request.targets(q);

		if (Robot.call(srv))
		{
			ROS_INFO("Sum: %ld", (long int)srv.response.success);
		}// if 
		else
		{
			ROS_ERROR("Failed to call service add_two_ints");
		}// else

		//ros::Duration(0.5).sleep(); // Sleep for half a second (Does ROS operates in seconds or miliseconds?)

	}// for

	*/
		
	
	
	//ros::spinOnce(); // ros::spin(); //The program will be finished with 'ctrl+c' keyboard interrupt.
 	return 0;

} // main
