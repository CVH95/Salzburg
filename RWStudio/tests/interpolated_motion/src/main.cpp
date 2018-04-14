// EXERIMENTING WITH CUBIC TRAJECTORY INTERPOLATION


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
USE_ROBWORK_NAMESPACE
using namespace robwork;
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




void luascript(QPath path, const string deviceName) 
{

	ofstream luafile;
	stringstream ss;
	string LuaFilePath = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/tests/interpolated_motion/genfiles/trajectory.lua";
	
	luafile.open(LuaFilePath);

	luafile << "wc = rws.getRobWorkStudio():getWorkCell()" << endl;
	luafile << "state = wc:getDefaultState()" << endl; 
	luafile << "device = wc:findDevice(\"" << deviceName << "\")" << endl; 
	luafile << "   " << endl;
	luafile << "function setQ(q)" << endl;
	luafile << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])" << endl;
	luafile << "device:setQ(qq,state)" << endl;
	luafile << "rws.getRobWorkStudio():setState(state)" << endl;
	luafile << "rw.sleep(0.1)" << endl;
	luafile << "end" << endl;
	luafile << "   " << endl;
	luafile << "   " << endl;
	luafile << "setQ({-1.04475, -1.78024, -2.24013, -0.781733, 1.61513, 0.0356047})" << endl;
	
	for (QPath::iterator it = path.begin(); it < path.end(); it++)  
	{
		Q currentStep = *it;
		luafile << "setQ({" << currentStep[0] << "," << currentStep[1] << "," << currentStep[2] << "," << currentStep[3] << "," << currentStep[4] << "," << currentStep[5] <<"})" << endl;	
	}	
	
	// luafile << "attach(bottle,table)" << endl;
	// luafile << "setQ({1.571, 0.006, 0.03, 0.153, 0.762, 4.49})" << endl;

	luafile.close();

}


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char *argv[]) 
{
	Math::seed();
	RobWork::getInstance()->initialize();
	Log::log().setLevel(Log::Info);


	cout << "CUBIC TRAJECTORY INTERPOLATION" << endl;
	cout << endl;
	cout << "	>> Read planned path from file" << endl;
	cout << "	>> Interpolate cubic trajectory given all path points." << endl;
	cout << "	>> Generate .lua file for simulation in RobWorkStudio." << endl;
	cout << endl;
	cout << endl;


	// Defining variables
	QPath path;
	vector<double> times;
	double tt = 0.1;
	rw::math::Q v_start(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	rw::math::Q v_end(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

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

		cout << q_new << endl;
		
		// Store entry
		path.push_back(q_new);
		times.push_back(tt);

		ind = ind + 6;
		tt = tt + 0.1;

	} // for i 


	cout << endl;

	// Interpolating trajectory:
	const string dvc = "UR1";
	QPath interpolated_path;
	InterpolatorTrajectory<rw::math::Q>::Ptr traj = CubicSplineFactory::makeClampedSpline(path, times, v_start, v_end);

	/* check out the results */
  	double t0 = traj->startTime();
  	double tn = traj->endTime();

	// Displaying & saving trajectory.
	ofstream fss;
	fss.open("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/tests/interpolated_motion/genfiles/trajectory.txt");

	cout << "Interpolated trajectory (shown Q vectors for each 0.01 s):" << endl;
	cout << endl;
	cout << endl;   
  	for (double t = t0; t <= tn; t += 0.01) 
	{

		// Display
    		cout << "| t = "  << t << " |      ··· Q = (" << traj->x(t)[0] << ", " << traj->x(t)[1] << ", " << traj->x(t)[2] << ", " << traj->x(t)[3] << ", " << traj->x(t)[4] << ", " << traj->x(t)[5] << ") ···" << endl;  

		// Save to file
		fss  << traj->x(t)[0] << " " << traj->x(t)[1] << " " << traj->x(t)[2] << " " << traj->x(t)[3] << " " << traj->x(t)[4] << " " << traj->x(t)[5] << endl;  		

		// Conversion to save to .lua file
		rw::math::Q q_i(6, traj->x(t)[0], traj->x(t)[1], traj->x(t)[2], traj->x(t)[3], traj->x(t)[4], traj->x(t)[5] );  
		interpolated_path.push_back(q_i);

	} // for 

	cout << endl;
	cout << "interpolated_path vector length: " << interpolated_path.size() << endl; // "    |    Trajectory length: " << interpolated_path.size()/6 << endl;
	luascript(interpolated_path, dvc);

	cout << endl;
	cout << "_EOF_" << endl;
	cout <<"  ." << endl;
	cout <<"  ." << endl;

	return 0;

} // main
	
