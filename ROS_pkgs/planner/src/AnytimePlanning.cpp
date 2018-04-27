// Anytime Dynamic Path Planning Shared Library

// Implemented fr Robotics & Computer Vision 2 course 2018
// SDU Robotics
// Carlos, Sergi, Richárd & Mathesh

#include <AnytimePlanning.h>

#define MAXTIME 100.

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


/* NOTE */


void AnytimePlanning::Load_WorkCell(const string wc_name, const string dev_name)
{

	cout << "	>> WorkCell: " << wc_name << endl; 

	// if found, loading workcell
	wc = WorkCellFactory::load(wc_name);
	if (wc == NULL) 
	{
		cerr << "WorkkCell: " << wc_name << " not found!" << endl;
		wc_found = false;
	} // if	

	// and device
	device = wc->findDevice(dev_name);
	if (device == NULL) 
	{
		cerr << "Device: " << dev_name << " not found!" << endl;
		dev_found = false;
	} // if

	cout << "	>> Found device: " << dev_name << endl;

}// Load_WorkCell()


/* 
	In the main, after calling Load_Workcell(), use the following breaking condition:

	if(wc_found == false || dev_found == false)
		{return 0;}
*/


// Function that looks for collisions at a given state (Q).
bool AnytimePlanning::checkCollisions(const State &state, const CollisionDetector &detector, const rw::math::Q &q) 
{
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

} // checkCollisions()



// Constraint and Collision strategies; Then Generates path
QPath AnytimePlanning::get_path(double epsilon, rw::math::Q from, rw::math::Q to)
{
	// Get state
	State state = wc->getDefaultState();

	// Set collision detection strategy.
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()); 

	// Set the planner constraint to build RRT-connect.
	PlannerConstraint constraint = PlannerConstraint::make(&detector, device, state); 
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

	// Check for collisions at initial configuration.
	if (!AnytimePlanning::checkCollisions(state, detector, from))
		{return 0;}
	// Check for collisions at final configuration.
	if (!AnytimePlanning::checkCollisions(state, detector, to))
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
	pf.open("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_original.txt");
	for (QPath::iterator it = path.begin(); it < path.end(); it++) 
	{
		pf << j << ":  " << *it << endl;
		j++;	
	} // for
	cout << "Saved to /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_original.txt" << endl;
	cout << endl;
	
	pf.close();

	return path;

} // get_path()




// Interpolate path nodes to get a trayectory
QPath AnytimePlanning::get_trajectory(QPath path, rw::math::Q dq_start, rw::math::Q dq_end)
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
	tf.open("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_interpolated.txt");	

	double t0 = traj->startTime();
  	double tn = traj->endTime();

	cout << "	>> Trajectory duration: " << tn << " seconds." << endl;
	
	for (double t = t0; t <= tn; t += 0.01) 
	{
		// Conversion to save to .lua file
		rw::math::Q q_i(6, traj->x(t)[0], traj->x(t)[1], traj->x(t)[2], traj->x(t)[3], traj->x(t)[4], traj->x(t)[5] );  
		tf << j << ":  t = " << t << "  |  " << q_i << endl;
		interpolated_path.push_back(q_i);

	} // for
	cout << "	>> Trajectory length: " << interpolated_path.size() << " nodes." << endl;
	cout << "	>> Saved with time steps of 0.01 seconds." << endl;
	cout << endl;
	cout << "Saved to /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_interpolated.txt" << endl;
	cout << endl;
	
	tf.close();
	
	return interpolated_path;

} // get_trajectory()



// This function reads the path to go back to q_start from the file where it is stored. This path is always constant in the program.
// No obstacle are added when returning.
QPath AnytimePlanning::return_path(const string filename)
{

	ifstream rf;
	rf.open(filename);

	string lines;	
	int nl = 0;

	QPath return_path;

	// Getting number of lines in .csv file (number of steps in the path).
	while( getline(rf, lines) )
	{
		nl++;
	
	} // while 
	
	rf.close();


	// Transfering joint values.
	ifstream rff(filename);
	float val;
	vector<float> values;
	
	while( rff >> val )
    	{
	
		values.push_back(val);
    	
	} // for line
		
	rff.close();

	int values_lgth = values.size()/6;

	if (nl == values_lgth)
	{
		// Reading data in file into rw::math::Q vector to be stored in QPath.
		cout << "	>> Reading backwards path." << endl;
		cout << endl;
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
		
			return_path.push_back(q_new);
		
			ind = ind + 6;

		} // for i

	} // if
	else
	{
		cout << "An error occurred while reading the file" << endl;
		return 0;
	} // else

	return return_path;

} // return_path()


// Function to send a trajectory requesting the CAROS service move_servo_q
void AnytimePlanning::send_trajectory(ros::ServiceClient client, caros_control_msgs::SerialDeviceMoveServoQ srv, QPath path)
{

	for (const rw::math::Q& p : path)
	{
		// This should be able to move the robot
		
		caros_common_msgs::Q q;
		q = caros::toRos(p);
		srv.request.targets.push_back(q);
    		srv.request.speeds.push_back(0.001);

		if (client.call(srv))
		{
			ROS_INFO("CAROS_MOVE_SERVO_Q RESPONSE: %d", srv.response.success);
		}// if
		else	
		{
		ROS_ERROR("ERROR IN CAROS_MOVE_SERVO_Q REQUEST. FAILED TO MOVE ROBOT");
		}// else

		//ros::Duration(0.5).sleep(); // Sleep for half a second (Does ROS operates in seconds or miliseconds?)

	}// for	

}// send_trajectory()

