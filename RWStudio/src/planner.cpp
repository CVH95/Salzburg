#include <iostream>
#include <fstream> 
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <vector>
#include <math.h> 
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <string>
#include <vector>
#include <rw/trajectory/CubicSplineFactory.hpp>

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


#define MAXTIME 1000.


// GLOBAL VARIABLES
rw::pathplanning::QToQPlanner::Ptr planner;
//bool ext1 = false;
//bool ext2 = false;

//---------------------------------------------------------------------------------------------------------


// Function that looks for collisions at a given state (Q).
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) 
	{
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) 
		{
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		} //for
		return false;
	} // if
	return true;
} // check_collisions



// Constraint and Collision strategies; Then Generates path
QPath get_path(WorkCell::Ptr wc, Device::Ptr device, double epsilon, State state, rw::math::Q from, rw::math::Q to)
{
	// Set collision detection strategy.
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()); 

	// Set the planner constraint to build RRT-connect.
	PlannerConstraint constraint = PlannerConstraint::make(&detector, device, state); 
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

	// Check for collisions at initial configuration.
	if (!checkCollisions(device, state, detector, from))
		{return 0;}
	// Check for collisions at final configuration.
	if (!checkCollisions(device, state, detector, to))
		{return 0;}

	/*if(ext1 == true || ext2 == true)
		{return 0;}*/

	cout << endl;
	cout << "Initial and Final configurations checked. <=========> Collision Free" << endl;
	cout << endl;

	planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, RRTPlanner::RRTConnect);

	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from, to, path, MAXTIME);
	t.pause();
	cout << endl;
	cout << "	>> Path's length: " << path.size() << endl;
	cout << "	>> Computation time: " << t.getTime() << " seconds." << endl;
	cout << endl;

	// Saving original plan
	int j = 0;
	ofstream pf;
	pf.open("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/backwards_path.txt");
	for (QPath::iterator it = path.begin(); it < path.end(); it++) 
	{
		pf << j << ":  " << *it << endl;
		j++;	
	} // for
	cout << "Saved to /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/backwards_path.txt" << endl;
	cout << endl;
	
	pf.close();

	return path;

} // get_path()


// Interpolate path nodes to get a trayectory
QPath get_trajectory(QPath path, rw::math::Q dq_start, rw::math::Q dq_end)
{

	int imax = path.size();
	QPath interpolated_path;
	vector<double> times;
	double tt = 0.1;
	for( int i = 0; i< imax; i++)
	{
		times.push_back(tt);
		tt = tt+0.1;
	} // for
	InterpolatorTrajectory<rw::math::Q>::Ptr traj = CubicSplineFactory::makeClampedSpline(path, times, dq_start, dq_end);

	int j = 0;
	ofstream tf;
	tf.open("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/backwards_path_interpolated.txt");
	ofstream sf;
	sf.open("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/backwards_trajectory");	

	double t0 = traj->startTime();
  	double tn = traj->endTime();

	cout << "	>> Trajectory duration: " << tn << " seconds." << endl;
	
	for (double t = t0; t <= tn; t += 0.01) 
	{
		// Conversion to save to .lua file
		rw::math::Q q_i(6, traj->x(t)[0], traj->x(t)[1], traj->x(t)[2], traj->x(t)[3], traj->x(t)[4], traj->x(t)[5] );  
		tf << j << ":  t = " << t << "  |  " << q_i << endl;
		sf << traj->x(t)[0] << " " << traj->x(t)[1] << " " << traj->x(t)[2] << " " << traj->x(t)[3] << " " << traj->x(t)[4] << " " << traj->x(t)[5] << endl;
		interpolated_path.push_back(q_i);	
		j = j+1;

	} // for
	cout << "	>> Trajectory length: " << interpolated_path.size() << " nodes." << endl;
	cout << "	>> Saved with time steps of 0.01 seconds." << endl;
	cout << endl;
	cout << "Saved to /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/backwards_path_interpolated.txt" << endl;
	cout << endl;
	
	tf.close();
	sf.close();

	return interpolated_path;

} // get_trajectory()


//---------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	rw::math::Math::seed();
	
	// LOADING WORKCELL 

	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";
	cout << "WorkCell " << wcFile; //<< " and device " << deviceName << endl;

	// if found, loading workcell
	WorkCell::Ptr wc = WorkCellFactory::load(wcFile);	
	// and device
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) 
	{
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	} // if


	// INITIAL SETTINGS

	// Initial configuration defined		
	Q from_deg(6, -59.86, -102.00, -128.35, -44.79, 92.54, 2.04); // deg
	//Q from_deg(6, -50.85, -108.53, -112.61, -44.85, 84.21, 28.59); // deg
	Q from = from_deg*(3.14159265359/180); // rad
	// Goal configuration defined.
	Q to_deg(6,151.34, -96.45, -107.53, -120.87, 272.71, 2.02); // deg
	//Q to_deg(6, -99.47, -164.99, -12.12, -104.93, 84.24, 28.59); // deg
	Q to = to_deg*(3.14159265359/180); // rad

	cout << endl;
	cout << "	>> | Q_initial | = " << from << endl;
	cout << "	>> | Q_goal | = " << to << endl;
	cout << endl;	

	// Initiate state variable by default.
	State state = wc->getDefaultState(); 
	// Get Home position
	Q Q_home = device->getQ(state);
	// Set new state to Q_init position.
	device->setQ(from,state);


	// PLANNING

	double epsilon = 0.5; // RRT's extend parameter.
	//double timeStep = 0.1; // Time step between path nodes to interpolate them.
	rw::math::Q dq_start = rw::math::Q(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	rw::math::Q dq_end = rw::math::Q(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	// Create planner
	/*create_planner(wc, device, epsilon, state, from, to);
	if(ext1 == true || ext2 == true)
		{return 0;}*/
	// Calculate path
	QPath rawPath = get_path(wc, device, epsilon, state, to, from);
	// Interpolate path
	QPath smoothPath = get_trajectory(rawPath, dq_start, dq_end);

	return 0;

} // main
