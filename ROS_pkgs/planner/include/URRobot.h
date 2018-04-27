// Created from ur_caros_example given in the ROS Lecture.

#include "rw/rw.hpp"
#include "caros/serial_device_si_proxy.h"
#include "ros/package.h"
#include <iostream>

class URRobot {
	using Q = rw::math::Q;

private:
	rw::models::WorkCell::Ptr wc_;
	rw::models::Device::Ptr device_;
	rw::kinematics::State state_;
	caros::SerialDeviceSIProxy* robot;

public:
	URRobot(ros::NodeHandle nh);
	Q getQ();
	
};
