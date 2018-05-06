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

	state = wc->getDefaultState();
	AnytimePlanning::add_red_ball(0.5);
	
	// See if the obstacle was found
	obstacle = wc->findObject("RedBall");
	if (obstacle == NULL) 
	{
		cerr << "Red Ball not found in the WorkCell!" << endl;
		dev_found = false;
	} // if

	cout << "	>> Red Ball created and added succesfully to the WorkCell." << endl;
	
	// Set the ball in a starting position
	AnytimePlanning::move_red_ball(0.0, 0.0, 0.0);

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
		return true;
	} // if
	return false;

} // checkCollisions()



// Function to save a path to a file
void AnytimePlanning::save_path(const string filename, QPath path)
{

	ofstream f;
	f.open(filename);
	for( const rw::math::Q& q : path )
	{

		f << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << endl; 

	}// for

	f.close();

}// save_path()


QPath AnytimePlanning::read_path(const string filename)
{
	QPath path;

	ifstream f;
	f.open(filename);

	string lines;
	int nl = 0;

	// Getting number of lines in file (number of steps in the path).
	while( getline(f, lines) )
	{
		nl++;
	
	} // while 
	
	f.close();

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
		
			path.push_back(q_new);
		
			ind = ind + 6;

		} // for i

	} // if
	else
	{
		cout << "An error occurred while reading the file" << endl;
		return 0;

	} // else

	return path;

}// read_path()


// Function to create the ball in the WokCell
void AnytimePlanning::add_red_ball(double radius)
{
	
	// Adding frames to workcell: from ROVI1 project
	const string ball = "RedBall";
	const string child_of = "WORLD";

	ball_frame = new rw::kinematics::MovableFrame(ball);
	rw::kinematics::Frame* parent = wc->findFrame(child_of);
	wc->addFrame(ball_frame, parent);
	
	// Update state in the workcell to finally add the frame
	state = wc->getStateStructure()->upgradeState(state);
        wc->getStateStructure()->setDefaultState(state);

	// Adding ball geometry (10% larger in radius than the actual ball to add some clearence)
	rw::geometry::Geometry::Ptr ball_geometry = rw::loaders::GeometryFactory::load( "#Sphere 1.1*radius" );
	ball_geometry->setFrame(ball_frame);

	rw::models::RigidObject::Ptr obs = new rw::models::RigidObject(ball_frame);
	wc->add(obs);

} // add_red_ball()


// To use the ball as an obstacle, we introduce a model in the WorkCell and move it to the coordinates given by the stereo nodes.
// Important to have a good calibration relative to the Robot's base.
// Function to move the ball to the coordinates given by the Vision node
void AnytimePlanning::move_red_ball(float X, float Y, float Z)
{
	double x = (double) X;
	double y = (double) Y;
	double z = (double) Z;

	rw::kinematics::Frame* Ref = device->getBase();
	rw::math::Transform3D<double> ballToBase;
	ballToBase.P() = rw::math::Vector3D<double>(x, y, z);
	rw::math::RPY<double> rpy = rw::math::RPY<double>(0, 0, 0);
	ballToBase.R() = rpy.toRotation3D();
	// void moveTo 	( const math::Transform3D<> &  transform, Frame *refframe, State &  state ) 
	// Changes the transform in the state, such that the movable frame is located in the transform which is described relative to refframe.
	ball_frame->moveTo(ballToBase, Ref, state); 


}// move_red_ball()


// Function that gets the coordinates of the ball from ROS network.
void AnytimePlanning::ball_location_callback(const geometry_msgs::PointStamped::ConstPtr &msg )
{

	AnytimePlanning::move_red_ball(msg->point.x, msg->point.y, msg->point.z);


}// ball_location_callback



void AnytimePlanning::find_obstacles(ros::NodeHandle nh, const string topic)
{
	ros::Subscriber sub = nh.subscribe(topic, 1, &AnytimePlanning::ball_location_callback, this);
	state = wc->getDefaultState();
		

}// find_obstacles()




bool AnytimePlanning::invalidate_nodes(QPath path, ros::NodeHandle nh, const string topic)
{

	AnytimePlanning::find_obstacles(nh, topic);
	// Set collision detection strategy.
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()); 
	bool colliding;
	for (const rw::math::Q& q : path)
	{
		// Check collision		
		colliding = AnytimePlanning::checkCollisions(state, detector, q);
		if (colliding == true)
		{
			
			q_collision = q;
			cout << "Collision detected in Q = " << q_collision << endl;
			return true;
		}// if

	}// for

	return false;

}// invalidate_nodes()



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



QPath AnytimePlanning::replan(rw::math::Q q_stop, rw::math::Q q_goal, rw::math::Q dq_start, rw::math::Q dq_end, const string filename, double epsilon)
{

	QPath path = AnytimePlanning::get_path(epsilon, q_stop, q_goal);
	AnytimePlanning::save_path(filename, path);
	QPath traj = AnytimePlanning::get_trajectory(path, dq_start, dq_end);

	return traj;


}// rePlan()



// This function reads the path to go back to q_start from the file where it is stored. This path is always constant in the program.
// No obstacle are added when returning.
QPath AnytimePlanning::return_path(const string filename)
{

	QPath return_path = AnytimePlanning::read_path(filename);

	return return_path;

} // return_path()


// Function to send a trajectory requesting the CAROS service move_servo_q (offline path planning)
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


// Collision status msg callback
void AnytimePlanning::collision_callback(const std_msgs::Bool::ConstPtr &status)
{
	if(status->data == true)
	{
		collision_status = true;
	}// if
	else
	{
		collision_status = false;
	}

}// collision_callback()

// Online path planning: Send trajectory + collision checking + replanning
void AnytimePlanning::dynamic_trajectory(ros::NodeHandle nh, QPath path, double e, ros::ServiceClient client, caros_control_msgs::SerialDeviceMoveServoQ srv, const string bool_t_n, Q goal, 
					Q dq0, Q dq1, const string filename)
{	
	URRobot UR5(nh);

	ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(bool_t_n, 1, &AnytimePlanning::collision_callback, this);
	
	while(collision_status == false)
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

		}// for

	}// while

	if(collision_status == true)
	{

		rw::math::Q q_now = UR5.getQ();
		QPath new_path = AnytimePlanning::replan(q_now, goal, dq0, dq1, filename, e);
		AnytimePlanning::dynamic_trajectory(nh, new_path, e, client, srv, bool_t_n, goal, dq0, dq1, filename);
	
	}// if inCollision

}//dynamic_trajectory()
