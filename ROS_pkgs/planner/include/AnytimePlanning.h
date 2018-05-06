// Anytime Dynamic Path Planning Shared Library

// Implemented fr Robotics & Computer Vision 2 course 2018, SDU Robotics
// Carlos, Sergi, Richárd & Mathesh
// Uses RobWork & CAROS

// AnytimePlanning.h

//Libs
#include <URRobot.h>
// General
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
// Caros & ROS headers
#include "ros/ros.h"
#include <caros/common.h>
#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>
#include "caros_control_msgs/SerialDeviceMoveServoQ.h"
#include "caros_common_msgs/Q.h"
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>



using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


class AnytimePlanning{

  public: 
	
	// Generañ
	void Load_WorkCell(const string wc_name, const string dev_name);
	bool checkCollisions(const State &state, const CollisionDetector &detector, const Q &q);
	void save_path(const string filename, QPath path);
	QPath read_path(const string filename);

	// Obstacle related
	void add_red_ball(double radius);
	void move_red_ball(float X, float Y, float Z);
	void ball_location_callback(const geometry_msgs::PointStamped::ConstPtr &msg );
	void find_obstacles(ros::NodeHandle nh, const string topic);
	bool invalidate_nodes(QPath path, ros::NodeHandle nh, const string topic);

	// Path-Planning related
	QPath get_path(double epsilon, rw::math::Q from, rw::math::Q to);
	QPath get_trajectory(QPath path, rw::math::Q dq_start, rw::math::Q dq_end);
	QPath return_path(const string filename);
	QPath replan(rw::math::Q q_stop, rw::math::Q q_goal, rw::math::Q dq_start, rw::math::Q dq_end, const string filename, double epsilon);

	// CAROS interface related
	void send_trajectory(ros::ServiceClient client, caros_control_msgs::SerialDeviceMoveServoQ srv, QPath path);
	void collision_callback(const std_msgs::Bool::ConstPtr &status);
	void dynamic_trajectory(ros::NodeHandle nh, QPath path, double e, ros::ServiceClient client, caros_control_msgs::SerialDeviceMoveServoQ srv, const string bool_t_n, Q goal, 
					Q dq0, Q dq1, const string filename);

  private: 
	/*const string wc_name;
	const string dev_name;*/
	rw::models::WorkCell::Ptr wc;
	Device::Ptr device;
	State state;
	Object::Ptr obstacle;
	bool dev_found;
	bool wc_found;
	rw::pathplanning::QToQPlanner::Ptr planner;
	rw::kinematics::MovableFrame* ball_frame;
	geometry_msgs::PointStamped location3D;
	rw::math::Q q_collision;
	bool collision_status;

}; // AnytimePlanning



/*

	Functions needed:

		1. send_path()	------> ROS 
		2. find_obstacles() (get_ball_position) ------> ROS
		3. update_obstacle_map()  -------> ROS
		4. invalidate_nodes()

*/
