// Kalman Shared Library

// ROVI2 Object Avoidance

// Richárd, Sergi, Mathesh and Carlos

// kalman_3d.h

// General 
#pragma once
#include <stdio.h>
#include <iostream>
#include <fstream>
//#include <array>
#include <string>
#include <vector>
// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PointStamped.h>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

// RobWork headers no funciona! :(
//#include <rw/rw.hpp>




using namespace std;
using namespace cv;


class KALMAN{

	private:

		//ros::Subscriber left_subs;
		//ros::Subscriber right_subs;
		ros::Subscriber triangulation;
		ros::Publisher pub;
		bool is_initialized;
		// Declare messages 		
		
		geometry_msgs::PointStamped location3D;
		geometry_msgs::PointStamped KF_location_3D;
		cv::KalmanFilter KF3;
		//float left0, left1;
		//float right0, right1;
		

	public:
		
		
		// Constructor
		KALMAN(ros::NodeHandle nh, const string pub_name);

		// Destructor
		virtual ~KALMAN();

		// Methods
		float calcU(float t, float noise );
		float calcX(float t, float noise );
		float calcY(float t, float noise );
		float calcZ(float t, float noise );

		KalmanFilter positionKF();
		KalmanFilter velocityKF();
		KalmanFilter accelerationKF();
		cv::Point3_<float> predict(cv::KalmanFilter &KF);
		cv::Point3_<float> correct(cv::KalmanFilter &KF, cv::Mat Measurement);
		void skipCorrect(cv::KalmanFilter &KF);
		float L2error(cv::Point pt1, cv::Point pt2);
		float L3error(cv::Point3_<float> pt1, cv::Point3_<float> pt2);	
		cv::Point3_<float> Kalman_filter_3D(cv::Mat measPt);

		void pub_kf(ros::NodeHandle nh, const string pub_name);
		void sub_kf(ros::NodeHandle nh, const string sub_name);
		void ball_location_kf_callback(const geometry_msgs::PointStamped::ConstPtr &msg);


		
};
