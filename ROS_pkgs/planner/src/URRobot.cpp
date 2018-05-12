#include <URRobot.h>


// This is the object constructor for the class URRobot. When the object is created in the main, it requires a ros::NodeHandle argument.
// See definition in main.cpp
URRobot::URRobot(ros::NodeHandle nh)
{
	
	wc_ = rw::loaders::WorkCellLoader::Factory::load("/home/richard/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml");
	device_ = wc_->findDevice("UR1");
	state_ = wc_->getDefaultState();
	robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

	// Wait for first state message, to make sure robot is ready
	ros::topic::waitForMessage<caros_control_msgs::RobotState>("/caros_universalrobot/caros_serial_device_service_interface/robot_state", nh);
    	ros::spinOnce();

}//URRobot()



// Function to get current configuration of the robot. 
rw::math::Q URRobot::getQ()
{
	// spinOnce processes one batch of messages, calling all the callbacks
	ros::spinOnce();
	rw::math::Q q = robot->getQ();
	device_->setQ(q, state_);
        return q;

}// getQ()
