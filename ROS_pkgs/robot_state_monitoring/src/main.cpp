// ROVI2 - Object avoidance: Robot state monitoring node

#include <iostream>
#include <string>
#include <vector>
// ROS headers
#include "ros/ros.h"
#include <caros/common.h>
#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>
// RobWork headers
#include <rw/rw.hpp>

using namespace std;

void chatterCallback(const caros_control_msgs::RobotState msg)
{
	cout << "--------------------------------------------------------" << endl;
	cout << endl;
	cout << "ROBOT STATE:" << endl;
	cout << msg << endl;
	cout << endl;

} //chatterCallback() 

//---------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{

	cout << "ROBOT STATE MONITORING CONTROL OF THE UR" << endl;
	
	ros::init(argc, argv, "robot_state_monitoring");
	ros::NodeHandle nh;

	ros::Subscriber RS = nh.subscribe("/caros_universalrobot/caros_serial_device_service_interface/robot_state", 1, chatterCallback);

	ros::spin(); // The program will be finished with 'ctrl+c' keyboard interrupt.
 	return 0;

} // main
