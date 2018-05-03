#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <fstream> // for writing to a file
#include <sstream> // for processing the string

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

#define MAXTIME 15.

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	// Init the state of the workcell (or kinematics tree)
	State testState;
	// CollisionDetector > implements an efficient way of checking a COMPLETE frame tree for collisions.
	// QueryResult > result of a collision question. Bool? 
	CollisionDetector::QueryResult data;
	bool colFrom;
	testState = state;
	// devicePTR smart pointer type to an abstract device class. The device class is the basis for all other devices.
	// It is assumed that all devices have a configuration which can be encoded by rw::math::Q that all have a base 
	// frame representing where in the work cell they are located and a primary end frame. Some devices have multiple
	// end frames.

	// Sets configuration vector (q) into the state defined.
	
	device->setQ(q,testState);

	// Check the workcell for collision. Three arguments : State 'IN' (for which to check collisions).
	// Result 'OUT' (Queryresult : If non-NULL, the pairs of colliding frames are inserted in result) 
	// stopAtFirstContact : If result is non-NULL and stopAtFirstContact is true, then only the first colliding pair is
	// inserted in result. By default all coilliding parts are inserted >> BOOL = FALSE
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		// FramePairSet : A set of frame pairs.
		FramePairSet fps = data.collidingFrames; // the frames that are colliding
		// FramePairSet::iterator >> Forward iterator for frames
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			// Gets the two frames colliding.
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

CollisionStrategy::Ptr testStrategy_sphere(const CollisionStrategy::Ptr& strategy, int radius)
{
    BOOST_TEST_MESSAGE("- Test Strategy Sphere");

    // strategy->clear();

    const Transform3D<> id = Transform3D<>::identity();

    MovableFrame* o1 = new MovableFrame("Object1", id);

    rw::kinematics::moveTo

    StateStructure tree;
    Frame* world = tree.getRoot();
    tree.addFrame(o1, world);

    State state = workcell.getDefaultState();
    cube1->setTransform(id, state);

    Geometry::Ptr geom = GeometryFactory::load( "#Sphere 1.1*radius" );
    strategy->addModel(o1,geom);
    return strategy;
    

}

int main(int argc, char** argv) {
	// Added a random seed
	rw::math::Math::seed();
	// Define filename workcell
	const string wcFile = "/home/sergi/rovi2_finalproject/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml";
	// Define device >> robot
	const string deviceName = "UR1";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;
	// Load the workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	// Load the device
	Device::Ptr device = wc->findDevice(deviceName);
	// Add Ball to the wc
	Object::Ptr ball;
	wc->add(ball);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	
	//const string bottle = "Bottle";
	//const string tool = "Tool";
	// Change with custom Q values
	Q from(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
	Q to(6,1.571, 0.006, 0.03, 0.153, 0.762, 4.49);

	State state = wc->getDefaultState();
	device->setQ(from,state);
	// Attach the bottle frame to the tool frame using gripFrame. This allow
	// us the tool to grip the bottle.
	// We must consider this for the planner!!

	//rw::kinematics::Kinematics::gripFrame(wc->findFrame(bottle),wc->findFrame(tool),state);

	// Collision detector for a workcell wc [IN] . The collision detector is
	// initialized with the strategy. Notice that the
	// collision detector will create and store models inside the strategy.
	// ProximityStrat... [in/out] function to create a default available 
	// collision strategy. Returns null if no collision
	// strategies are availble else a ptr to a collision strategy.
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
	/* A planner constraint is a small copyable object containing pointers 
	to a configuration constraint and an edge constraint.
	Sampling based path planners and path optimizers typically use a 
	PlannerConstraint object for the collision checking for the paths.
	A number of make() utility constructors are provided for applications where defaults
	for configuration and edge constraint can be used.
	Path are checked discretely for a default device dependent resolution.
	*/

	const CollisionStrategy::Ptr& strategy_sphere;
	// pasar punto centro esfera 2D imagen a 3D robot coordinates.
	strategy_sphere = testStrategy_sphere(strategy_sphere, radius);

	PlannerConstraint constraint = PlannerConstraint::make(strategy_sphere, workcell, device, state);

	// QSampler : Interface for the sampling configuration. 
	// QSampler::makeConstrained  A sampler filtered by a constraint. For 
	// each call of sample() up to maxAttemps configurations
	// are extracted from sampler and checked by constraint. The first 
	// sample that satisfies the constraint is returned, if no
	// such were found the empty configuration is returned.

	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform	(device),constraint.getQConstraintPtr());


	// Metrics on configurations. The constructor functions are parametrized
	// by a type Vector. Euclidean dist for vector types.
	// Distance between two points: dist[p and q] = sqrt(sum[(pi-qi)^2])

	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

	// Extend distance between the points of the configuration.

	double extend = 1;

	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;
	// QToQPlanner> Path Planner interface from start configuration to a 
	// goal configuration.
	/*
	RRT based point-to-point planner.

	Parameters
    constraint	[in] Constraint for configurations and edges.
    sampler	[in] Sampler of the configuration space.
    metric	[in] Metric for nearest neighbor search.
    extend	[in] Distance measured by metric by which to extend the tree 
   		     towards an attractor configuration.
    type	[in] The particular variation the RRT planner algorithm.
	*/
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint,sampler, metric, extend, RRTPlanner::RRTConnect);
	cout << "Planning from " << from << " to " << to << endl;
	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();
	cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
	if (t.getTime() >= MAXTIME) {
		cout << "Notice: max time of " << MAXTIME << 
		" seconds reached." << endl;
	}
	double texp =  t.getTime();
	int plength = path.size();
	for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		cout << *it << endl;
	}
	// Save the generated path to a .lua file
	time_t timeNow = time(0); // Current time
	struct tm * now = localtime( & timeNow );
	char timeBuffer [80];
     	strftime (timeBuffer,80,"path_%Y-%m-%d %H:%M:%S.lua",now);
     	std::ofstream LUAfile;
     	LUAfile.open (timeBuffer);
	LUAfile << "wc = rws.getRobWorkStudio():getWorkCell()\n";
	LUAfile << "state = wc:getDefaultState()\n";
	LUAfile << "device = wc:findDevice(\"Robot\")\n";
	//LUAfile << "gripper = wc:findFrame(\"Tool\");\n";
	//LUAfile << "bottle = wc:findFrame(\"Bottle\");\n";
	LUAfile << "table = wc:findFrame(\"Table\");\n";
	LUAfile << "computer = wc:findFrame(\"Computer\");\n";

	LUAfile << "function setQ(q)\n";
	LUAfile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n";
	LUAfile << "device:setQ(qq,state)\n";
	LUAfile << "rws.getRobWorkStudio():setState(state)\n";
	LUAfile << "rw.sleep(0.01)\n";
	LUAfile << "end\n";

	LUAfile << "function attach(obj, tool)\n";
	LUAfile << "rw.gripFrame(obj, tool, state)\n";
	LUAfile << "rws.getRobWorkStudio():setState(state)\n";
	LUAfile << "rw.sleep(1)\n";
	LUAfile << "end\n";


	LUAfile << "setQ({-3.142, -0.827, -3.002, -3.143, 0.099, -1.573})\n";
	LUAfile << "attach(bottle,gripper)\n";
  	
	for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		std::string itString = boost::lexical_cast<std::string>(*it);
		//Remove first 4 characters, e.g. Q[6]:
		itString = itString.erase(0,4);
		LUAfile << "setQ(" << itString << ")" <<endl;
	}
	LUAfile << "setQ({1.571, 0.006, 0.03, 0.153, 0.762, 4.49})\n";
	LUAfile << "attach(bottle,table)\n";
	LUAfile << "-- Path of length: " << plength << "\n" ;
	LUAfile << "-- Time elapsed: " << texp << "\n";

	LUAfile << "LUAfile.close();\n";

	// End of the .LUA script writing part ---------------------------------

	cout << "Program done." << endl;
	return 0;
}
