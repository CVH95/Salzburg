// ROVI2 - Object avoidance: Planner node

// Q_start   --->   Q[6]{-1.04475, -1.78024, -2.24013, -0.781733, 1.61513, 0.0356047} (rad)
// Q_goal    --->   Q[6]{2.64138, -1.68337, -1.87675, -2.10958, 4.75969, 0.0352557} (rad)

#include <AnytimePlanning.h>
//#include <URRobot.h>

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
#include "caros_control_msgs/SerialDeviceMoveServoQ.h"
//#include <caros/ur_service_interface.h>


using namespace std;

#define MAXTIME 1000.


//---------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "planner");
	ros::NodeHandle nh;
	
	rw::math::Math::seed();
	
	cout << "Ready to start" << endl;
	cout << endl;
	ros::Duration(0.5).sleep();


	// LOADING WORKCELL & INITIAL SETTINGS
	
	const string wcFile = "/home/richard/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2_with_ball/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";
	const string bw_file = "/home/richard/catkin_ws/src/ROVI2_Object_Avoidance/ROS_pkgs/planner/genfiles/backwards_trajectory.txt";
	const string pathFile = "/home/richard/catkin_ws/src/ROVI2_Object_Avoidance/tests/plan.txt";

	// Created object of the class URRobot with corresponding argument (ros::NodeHandle).
	URRobot ur2(nh);
	cout << "UR5's actual configuration:  " <<  ur2.getQ() << endl;
	cout << endl;

	// Created object of the class AnytimePlanning.
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

	// Set node to be client of "/caros_universalrobot/caros_serial_device_service_interface/move_servo_q".
	// In order to broadcast Q's belonging to the path, it is required that the client node sends Q request msgs to the server in charge of moving the robot. 
	ros::ServiceClient ur_client = nh.serviceClient<caros_control_msgs::SerialDeviceMoveServoQ>("/caros_universalrobot/caros_serial_device_service_interface/move_servo_q");
	caros_control_msgs::SerialDeviceMoveServoQ srv;

	// PATH PLANNING

	// Calculate path using RRT* algorithm
	QPath raw_path = plan.get_path(epsilon, from, to);
	// Interpolate the obtained path to create a smooth trajectory
	QPath trajectory = plan.get_trajectory(raw_path, dq_start, dq_end);
	cout << "trajectory ready" << endl;

	plan.save_path(pathFile, raw_path);
	plan.save_path("/home/richard/catkin_ws/src/ROVI2_Object_Avoidance/tests/trajectory.txt", trajectory);
	cout << "path saved" << endl;
	// Get trajectory to go back to q_start
	QPath bw_trajectory = plan.return_path(bw_file);
	
	// SEND (NUDES) PATH

	plan.send_trajectory(ur_client, srv, trajectory);

	cout << "Trajectory completed. Preparing to go back to the starting configuration." << endl;
	ros::Duration(3).sleep(); // Sleep for one and a half seconds (Does ROS operates in seconds or miliseconds?)


	plan.send_trajectory(ur_client, srv, bw_trajectory);
	
	cout << "Program completed successfully. Preparing to exit.... (ctrl+c)" << endl;
	ros::Duration(3).sleep();
		
	//ros::spinOnce(); ros::spin(); //The program will be finished with 'ctrl+c' keyboard interrupt.
	ros::spin(); //The program will be finished with 'ctrl+c' keyboard interrupt.
 	return 0;

} // main()
