// ROVI2 - Object Avoidance: Back to Q_start

#include <AnytimePlanning.h>
#include <URRobot.h>

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

// Include some kind of communication with anytime_planner to send bools 

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "planner");
        ros::NodeHandle nh;
	
	const string bw_file = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/ROS_pkgs/planner/genfiles/backwards_trajectory.txt";

	cout << "Going Backwards" << endl;

	ros::Duration(0.5).sleep();

	AnytimePlanning plan;
	
	// Start service to request motion to initial position.	
	ros::ServiceClient ur_client = nh.serviceClient<caros_control_msgs::SerialDeviceMoveServoQ>("/caros_universalrobot/caros_serial_device_service_interface/move_servo_q");
	caros_control_msgs::SerialDeviceMoveServoQ srv;
	
	QPath bw_trajectory = plan.return_path(bw_file);
	plan.send_trajectory(ur_client, srv, bw_trajectory);

	ros::spin();
	return 0;

}// main()
