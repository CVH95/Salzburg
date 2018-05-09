// ROVI2 - Object avoidance: Anytime Planner node

// Q_start   --->   Q[6]{-1.04475, -1.78024, -2.24013, -0.781733, 1.61513, 0.0356047} (rad)
// Q_goal    --->   Q[6]{2.64138, -1.68337, -1.87675, -2.10958, 4.75969, 0.0352557} (rad)

/*
	
	1. Loop.
		2. Initialize(T, q_initial, q_goal)
		3. path = planRRT(T)
		4. PostSolution(path)
		5. obstacles = findObstacles()
		6. for all obs found do:
			7. InvalidateNodes(T, obs)
		8. if path constains invalid nodes
			9. path = replanRRT(T)
			10. PostSolution(path)
		11. UpdateParameters(T).
*/

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
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
//#include <caros/ur_service_interface.h>


using namespace std;

#define MAXTIME 1000.


//---------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	ros::init(argc, argv, "anytime_planner");
	ros::NodeHandle nh;
	
	rw::math::Math::seed();

	// Files
	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2_with_ball/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";
	const string bw_file = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/ROS_pkgs/planner/genfiles/backwards_trajectory.txt";
	const string filename = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/tests/plan.txt";

	// Topics
	const string scene = "/red_ball_detection/triangulated_ball_location";
	const string stat = "/planner/collisions";


	// Created object of the class URRobot with corresponding argument (ros::NodeHandle).
        //URRobot ur2(nh);


	// INITIALIZE

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



	// PLANNING
	QPath path = plan.get_path(epsilon, from, to);
	plan.save_path(filename, path);
	QPath trajectory = plan.get_trajectory(path, dq_start, dq_end);

	plan.dynamic_trajectory(nh, trajectory, epsilon, ur_client, srv, stat, to, dq_start, dq_end, filename);
	
	ros::spin();
	return 0;

}// main()
	
