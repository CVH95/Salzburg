// ROBWORKSTUDIO SIMULATION OF INITIAL RRT* OFF-LINE PATH PLANNING


// Q_init = (-59.86, -102.00, -128.35, -44.79, 92.54, 2.04) [deg]
// Q_end = (151.34, -96.45, -107.53, -120.87, 272.71, 2.02) [deg]

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
 


// Display kinematic tree of the workcell on command line.
void printKinTree(Frame& printFrame, const State& printState, const Transform3D<>& parentTransform, int level)
{

	const Transform3D<> transform = parentTransform * printFrame.getTransform(printState);
	cout << level << ":  " << "|Frame name: " << printFrame.getName() << "|   " << "|Default State at " << transform.P() << "|" << endl;
	cout << endl;
	BOOST_FOREACH(Frame& child, printFrame.getChildren(printState))
	{
	cout << "            ";
	printKinTree(child, printState, transform, level+1);

	}	

}

// Function that looks for collisions at a given state (Q).
bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

void luascript(QPath path, int eps, const string deviceName) 
{

	ofstream luafile;
	stringstream ss;
	string LuaFilePath = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/clean_path.lua";
	
	luafile.open(LuaFilePath);

	luafile << "wc = rws.getRobWorkStudio():getWorkCell()" << endl;
	luafile << "state = wc:getDefaultState()" << endl; 
	luafile << "device = wc:findDevice(\"" << deviceName << "\")" << endl; 
	// luafile << "gripper = wc:findFrame(\"" << tool_frame_name << "\");" << endl; 
	// luafile << "bottle = wc:findFrame(\"" << bottle_frame_name << "\");" << endl; 
	// luafile << "table = wc:findFrame(\"" << table_frame_name << "\");" << endl; 
	luafile << "   " << endl;
	luafile << "function setQ(q)" << endl;
	luafile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])" << endl;
	luafile << "device:setQ(qq,state)" << endl;
	luafile << "rws.getRobWorkStudio():setState(state)" << endl;
	luafile << "rw.sleep(0.1)" << endl;
	luafile << "end" << endl;
	luafile << "   " << endl;
	// luafile << "function attach(obj, tool)" << endl;
	// luafile << "rw.gripFrame(obj, tool, state)" << endl;
	// luafile << "rws.getRobWorkStudio():setState(state)" << endl;
	// luafile << "rw.sleep(0.1)" << endl;
	// luafile << "end" << endl;
	// luafile << "   " << endl;
	luafile << "   " << endl;
	luafile << "setQ({-1.04475, -1.78024, -2.24013, -0.781733, 1.61513, 0.0356047})" << endl;
	// luafile << "attach(bottle,gripper)" << endl;
	
	for (QPath::iterator it = path.begin(); it < path.end(); it++)  
	{
		Q currentStep = *it;
		luafile << "setQ({" << currentStep[0] << "," << currentStep[1] << "," << currentStep[2] << "," << currentStep[3] << "," << currentStep[4] << "," << currentStep[5] <<"})" << endl;	
	}	
	
	// luafile << "attach(bottle,table)" << endl;
	// luafile << "setQ({1.571, 0.006, 0.03, 0.153, 0.762, 4.49})" << endl;

	luafile.close();

}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	rw::math::Math::seed();
	
	// LOADING WORKCELL AND GENERAL INFO

	cout << endl;	
	cout << "OFF-LINE PATH PLANNING WITH UR5" << endl;
	cout << "    ROVI2 Final Project." << endl;
	cout << "Richárd, Sergi, Mathesh & Carlos." << endl;
	// Looking for workcell
	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";
	//const string bottle_frame_name = "Bottle";
	const string tool_frame_name = "Computer";
	const string table_frame_name = "Table";
	
	cout << "WorkCell " << wcFile; //<< " and device " << deviceName << endl;

	// if found, loading workcell
	WorkCell::Ptr wc = WorkCellFactory::load(wcFile);	
	// and device
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}

	cout << "WorkCell contains the following devices:" << endl;
	cout << "  " << endl;
	BOOST_FOREACH(Device::Ptr dvcs, wc->getDevices()){
		cout << ">>  " << dvcs->getName() << endl;
	}
	cout << endl;
	cout << "------------------------------------------------------------------------------------" << endl;
	cout << endl;
	cout << "KINEMATIC TREE:" << endl;
	// Display kinematic tree for given workcell to see all frame names.
	printKinTree(*wc->getWorldFrame(), wc->getDefaultState(), Transform3D<>::identity(), 0);

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// INITIAL SETTINGS
	
	// Finding potential colliding frames
	// rw::kinematics::Frame* computer_frame =  wc->findFrame("Computer");
	// rw::kinematics::Frame* table_frame =  wc->findFrame("Table");

	// Initial configuration defined		
	Q from_deg(6, -59.86, -102.00, -128.35, -44.79, 92.54, 2.04); // deg
	//Q from_deg(6, -50.85, -108.53, -112.61, -44.85, 84.21, 28.59); // deg
	Q from = from_deg*(3.14159265359/180); // rad

	// Initiate state variable by default.
	State state = wc->getDefaultState(); 

	// Get Home position
	Q Q_home = device->getQ(state);
	
	cout << endl;
	cout << "	>> | Home | = " << Q_home << endl;
	cout << endl;
	

	// Set new state to Q_init position.
	device->setQ(from,state);

	cout << endl;
	cout << "	>> | Q_initial | = " << from << endl;
	cout << endl;	
	
	// Set collision detection strategy.
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()); 

	// Set the planner constraint to build RRT-connect.
	PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state); 
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

	// Set initial epsilon
	double epsilon = 0.5; // for RRT*

	// Goal configuration defined.
	Q to_deg(6,151.34, -96.45, -107.53, -120.87, 272.71, 2.02); // deg
	//Q to_deg(6, -99.47, -164.99, -12.12, -104.93, 84.24, 28.59); // deg
	Q to = to_deg*(3.14159265359/180); // rad
	
	cout << endl;
	cout << "	>> | Q_goal | = " << to << endl;
	cout << endl;

	// Check for collisions at initial configuration.
	if (!checkCollisions(device, state, detector, from))
		return 0;
	// Check for collisions at final configuration.
	if (!checkCollisions(device, state, detector, to))
		return 0;

	
	cout << endl;
	cout << "Initial and Final configurations checked. <=========> Collision Free" << endl;
	cout << endl;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// Execute off-line planning and return a valid path (no obstacle now)

	string FilePath = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/cleanPath.txt";
	ofstream file;
	file.open(FilePath);
	
	cout << "Starting planning" << endl;
	cout << "  ." << endl;	
	cout << "  ." << endl;
	cout << "  ." << endl;
	cout << endl;

	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, RRTPlanner::RRTConnect);

	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();
	
	cout << "Stopped:" << endl;
	cout << endl;
	cout << "	>> Path's length: " << path.size() << endl;
	cout << "	>> Computation time: " << t.getTime() << " seconds." << endl;
	cout << endl;
	cout << "Path sequence:" << endl;
	cout << endl;

	if (t.getTime() >= MAXTIME) {
		cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
	}

	// Print path in console
	for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		cout << *it << endl;
		Q currentStep = *it;
		file << currentStep[0] << " " << currentStep[1] << " " << currentStep[2] << " " << currentStep[3] << " " << currentStep[4] << " " << currentStep[5] << endl;
		
	}

	luascript(path, epsilon, deviceName);

	file.close();
	cout << endl;
	cout << endl;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// Experimenting on how to read the file
	ifstream inf("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/cleanPath.txt");
	string lines;
	int nl = 0;

	// Getting number of lines in .csv file (number of steps in the path).
	while( getline(inf, lines) )
	{
		nl++;
	
	} // while 
	
	cout << "	>> Path length = " << nl << "steps." << endl;
	cout << endl;
	inf.close();


	// Transfering joint values.
	ifstream fs("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/cleanPath.txt");
	float val;
	vector<float> values;
	
	while( fs >> val )
    	{
	
		values.push_back(val);
    	
	} // for line
		
	fs.close();

	int values_lgth = values.size();
	cout << "	>> Expected accumulator size: " << nl*6 << endl;
	cout << "	>> Actual aiccumulator size: " << values_lgth << endl;
	cout << endl;

	// Reading data in file into rw::math::Q vector to be transferred to the UR5.
	cout << "	>> Reading path:" << endl;
	int ind = 0;	
	for (int i = 0; i<nl; i++)
	{
		int joint0 = ind;
		int joint1 = ind+1;
		int joint2 = ind+2;
		int joint3 = ind+3;
		int joint4 = ind+4;
		int joint5 = ind+5;

		float q0 = values[joint0];
		float q1 = values[joint1];
		float q2 = values[joint2];
		float q3 = values[joint3];
		float q4 = values[joint4];
		float q5 = values[joint5];

		rw::math::Q q_new(6, q0, q1, q2, q3, q4, q5);
		
		cout << "Step " << i << ":     " << q_new << endl;
		// cout << "Step " << i << ": Q(" << i << ") = [" << q0 << "," << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5 << "]" << endl;
		
		ind = ind + 6;

	} // for i

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	// Experimenting with cubic trajectory interpolation 

	/*
	InterpolatorTrajectory<rw::math::Q>::Ptr trajectory;
	trajectory = CubicSplineFactory::makeNaturalSpline(path, 1.0);
	const string trajFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/Interpolated_Trajectory.txt";	
	ofstream file2;
	file2.open(trajFile);
	file2 << trajectory;
	file2.close();
	*/

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	cout << endl;
	cout << "_EOF_" << endl;
	cout << "  ." << endl;
	cout << "  ." << endl;
	cout << "  ." << endl;
	cout << "  ." << endl;
	cout << "  ." << endl;
	cout << "  ." << endl;
	cout << "Exiting..." << endl;
	return 0;
}
