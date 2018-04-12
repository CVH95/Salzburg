#include <iostream>
#include <vector>
#include <rw/rw.hpp>


using namespace std;
USE_ROBWORK_NAMESPACE
using namespace robwork;


int main(int argc, char *argv[]) 
{
	Math::seed();
	RobWork::getInstance()->initialize();
	Log::log().setLevel(Log::Info);
    
  	/* Original data 

  	QPath path;
  	vector<double> times;
  
  	path.push_back(Q(1, 0.0)); times.push_back(0.0);
  	path.push_back(Q(1, 1.0)); times.push_back(1.0);
  	path.push_back(Q(1, 0.0)); times.push_back(2.0);
  	path.push_back(Q(1, 2.0)); times.push_back(3.0);
  
  	Q v1(1, 0.0);
  	Q vn(1, 0.0);

	*/

	cout << "TRAJECTORY GENERATION: INTERPOLATION BY CUBIC SPLINES" << endl;
	cout << endl;
	cout << "Given:" << endl;
	cout << "	>> A set of joint configuration points (Q{6} vectors) in the path." << endl;
	cout << "	>> Associated passing times to each of these points." << endl;
	cout << "	>> Initial and final velocities of the trajectory." << endl;
	cout << endl;
	cout << endl;
   
  
	// Data with rw::math::Q vector with 6 DOF.

	QPath path;
	vector<double> times;

	rw::math::Q q1(6, -1.04475, -1.78024, -2.24013, -0.781733, 1.61513, 0.0356047); 
	rw::math::Q q2(6, -1.0162, -1.19242, -1.75018, -0.423582, 1.119, 0.0351778);
	rw::math::Q q3(6, 0.321194, -0.769924, -1.51916, -0.911447, 2.57768, 0.648952);
	rw::math::Q q4(6, 1.37718, -1.71639, -2.00076, -1.65401, 3.6777, 0.0368182);
	rw::math::Q q5(6, 2.64138, -1.68337, -1.87675, -2.10958, 4.75969, 0.0352557);

	path.push_back(q1);
	path.push_back(q2);
	path.push_back(q3);
	path.push_back(q4);
	path.push_back(q5);

	times.push_back(1.0);
	times.push_back(2.0);
	times.push_back(3.0);
	times.push_back(4.0);
	times.push_back(5.0);
	
	rw::math::Q v1(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	rw::math::Q vn(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	
	cout << "Configuation vectors  |  passing times:" << endl;
	cout << endl;
	cout << "	>> Q1 = " << q1 << "	| t = " << 1.0 << endl;
	cout << "	>> Q2 = " << q2 << "	| t = " << 2.0 <<  endl;
	cout << "	>> Q3 = " << q3 << "	| t = " << 3.0 <<  endl;
	cout << "	>> Q4 = " << q4 << "	| t = " << 4.0 <<  endl;
	cout << "	>> Q5 = " << q5 << "	| t = " << 5.0 <<  endl;
	cout << endl;
	cout << "Initial and final velocities dQ/dt:" << endl;
	cout << endl;
	cout << "	>> dQ_start = " << q1 << endl;
	cout << "	>> dQ_end = " << q2 << endl;
	cout << endl;
	cout << endl;
		
  	/* make spline trajectory */
  	InterpolatorTrajectory<rw::math::Q>::Ptr traj = CubicSplineFactory::makeClampedSpline(path, times, v1, vn);
  
  	/* check out the results */
  	double t0 = traj->startTime();
  	double tn = traj->endTime();

	// Displaying trajectory.

	cout << "Interpolated trajectory (shown Q vectors for each 0.01 s):" << endl;
	cout << endl;
	cout << endl;   
  	for (double t = t0; t <= tn; t += 0.01) 
	{

    		cout << "| t = "  << t << " |      ··· Q = (" << traj->x(t)[0] << ", " << traj->x(t)[1] << ", " << traj->x(t)[2] << ", " << traj->x(t)[3] << ", " << traj->x(t)[4] << ", " << traj->x(t)[5] << ") ···" << endl;    
//Add to display also velocity and acceleration:     <<   dQ/dt = " << traj->dx(t)[0] << "  >>" <<  endl; //traj->ddx(t)[0] << endl;
  
	} // for
  
  	return 0;

} // main
