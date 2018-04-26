// Anytime Dynamic Path Planning Shared Library

// Implemented fr Robotics & Computer Vision 2 course 2018, SDU Robotics
// Carlos, Sergi, Richárd & Mathesh
// Uses RobWork & CAROS

// AnytimePlanning.h

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
// Caros headers
//#include <caros/universal_robots.h>
#include <caros/common.h>
#include <caros/serial_device_si_proxy.h>
#include <caros/common_robwork.h>
#include <caros/ur_service_interface.h>


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
	
	void Load_WorkCell(const string wc_name, const string dev_name);
	bool checkCollisions(const State &state, const CollisionDetector &detector, const Q &q);
	QPath get_path(double epsilon, State state, rw::math::Q from, rw::math::Q to);
	QPath get_trajectory(QPath path, rw::math::Q dq_start, rw::math::Q dq_end);
	QPath return_path(const string filename);
	vector<caros::caros_common_msgs::Q> convert_trajectory(QPath path);

  private: 
	/*const string wc_name;
	const string dev_name;*/
	rw::models::WorkCell::Ptr wc;
	Device::Ptr device;
	State state;
	bool dev_found;
	bool wc_found;
	rw::pathplanning::QToQPlanner::Ptr planner;



}; // AnytimePlanning



/*

	Functions needed:

		1. send_path()	------> ROS 
		2. find_obstacles() (get_ball_position) ------> ROS
		3. update_obstacle_map()  -------> ROS
		4. invalidate_nodes()

*/
