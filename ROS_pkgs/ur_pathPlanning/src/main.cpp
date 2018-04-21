// ROVI2 final Project
// from ur_caros_example pkg

#include "rw/rw.hpp"
#include "caros/serial_device_si_proxy.h"
#include "ros/package.h"
#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <sstream>
#include <vector>

using namespace std;

class URRobot {
	using Q = rw::math::Q;

private:
	ros::NodeHandle nh;
	rw::models::WorkCell::Ptr wc;
	rw::models::Device::Ptr device;
	rw::kinematics::State state;
	caros::SerialDeviceSIProxy* robot;

public:
	URRobot()
	{
		auto packagePath = ros::package::getPath("ur_pathPlanning");
		wc = rw::loaders::WorkCellLoader::Factory::load("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml");
		device = wc->findDevice("UR1");
		state = wc->getDefaultState();
		robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

		// Wait for first state message, to make sure robot is ready
		ros::topic::waitForMessage<caros_control_msgs::RobotState>("/caros_universalrobot/caros_serial_device_service_interface/robot_state", nh);
	    ros::spinOnce();
	}

	Q getQ()
	{
		// spinOnce processes one batch of messages, calling all the callbacks
	    ros::spinOnce();
	    Q q = robot->getQ();
		device->setQ(q, state);
	    return q;
	}

	bool setQ(Q q)
	{
		// Tell robot to move to joint config q
        float speed = 0.5;
		if (robot->moveServoQ(q, speed)) {
            // moveServoQ temintates before the robot is at the destination, so wait until it is
			Q qCurrent;
			do {
				ros::spinOnce();
				qCurrent = robot->getQ();
			} while((qCurrent - q).norm2() > 0.01);
			device->setQ(q, state);
			return true;
		} else
			return false;
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "URRobot");
	URRobot robot;
	const string file = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/tests/interpolated_motion/genfiles/trajectory.txt";
	

	cout << "OPENING WORKCELL 2" << endl;
	cout << endl;
	cout << "	>> Device: UR5" << endl;
	cout << "	>> Current joint config:" << std::endl << robot.getQ() << std::endl << std::endl;

	cout << "	>> Reading path from file:" << std::endl;
	cout << file << endl;
	cout << endl;
	
	
	ifstream inf("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/tests/interpolated_motion/genfiles/trajectory.txt");
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
	ifstream fs("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/tests/interpolated_motion/genfiles/trajectory.txt");
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
		
		if (robot.setQ(q_new))
			std::cout << "| Step " << i << "succeded |" << endl;// "New joint config:" << std::endl << robot.getQ() << std::endl;
		else
			std::cout << std::endl << "FAILED TO MOVE ROBOT" << std::endl;	
	

		ind = ind + 6;

	} // for i

/*	
	float q1, q2, q3, q4, q5, q6;
	std::cin >> q1 >> q2 >> q3 >> q4 >> q5 >> q6;
	rw::math::Q q(6, q1, q2, q3, q4, q5, q6);



	if (robot.setQ(q))
		std::cout << std::endl << "New joint config:" << std::endl << robot.getQ() << std::endl;
	else
		std::cout << std::endl << "Failed to move robot" << std::endl;
*/
	return 0;
}
