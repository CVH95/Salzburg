//#pragma once
#include <cmath>
// ROS 
#include <ros/ros.h>
#include <ObstacleDetection.h>
#include <kalman_3d.h>
// General
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>
#include <numeric>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "draw_utils.hpp"
#include <iostream>


#define X_0 0		// start position x
#define Y_0 0		// start position y
#define V_0 100		// start velocity
#define Z_0 0
#define ALPHA 1	// start angle
#define T_0 0		// initial time
#define DT 0.15		// timestep
#define U_0 0

#define X_NOISE 1
#define Y_NOISE 0
#define Z_NOISE 2
#define PROCESS_NOISE 0.01 //(Q_)
#define MEASUREMENT_NOISE 1 //(R_)
#define ERROR_COV_POST_NOISE 0.1 //(P_)
#define G 9.81
using namespace std;
using namespace cv;


KALMAN::KALMAN(ros::NodeHandle nh, const string pub_name){


	/*cv::KalmanFilter KF = KALMAN::positionKF();
	cv::KalmanFilter KF2 = KALMAN::velocityKF();
	cv::KalmanFilter KF3 = KALMAN::accelerationKF();

	std::vector<cv::Point3_ <float> > ground_truth;
	std::vector<cv::Point3_ <float> > measurementPoints;
	std::vector<cv::Point3_ <float> > KF3D_Points;
	std::vector<float> error_points;*/
	pub = nh.advertise<geometry_msgs::PointStamped>(pub_name, 1);
	bool is_initialized = false;
	

}

/**
* Destructor.
*/
KALMAN::~KALMAN() {}

void KALMAN::pub_kf(ros::NodeHandle nh, const string pub_name){
	pub = nh.advertise<geometry_msgs::PointStamped>(pub_name, 1);
	ros::spin();
}


// Executing all the find obstacles routine
void KALMAN::sub_kf(ros::NodeHandle nh, const string sub_name)
{
	
	ros::Subscriber subs = nh.subscribe(sub_name, 1, &KALMAN::ball_location_kf_callback, this);

	ros::spin();

}// find_obstacles()

float KALMAN::calcU(float t, float noise = 0)
{
	// Computing measurement points e.g. ball trajectory with noise x position
	// See formula in pdf
	float U = V_0 * cos(ALPHA) * t + U_0;
	return U;
}

float KALMAN::calcX(float t, float noise = 0)
{
	// Computing measurement points e.g. ball trajectory with noise x position
	// See formula in pdf
	float X = calcU(t) * cos(ALPHA) + noise;
	return X;
}


float KALMAN::calcY(float t, float noise = 0)
{
	// Computing measurement points e.g. ball trajectory with noise y position
	// See formula in pdf
	float Y = calcU(t) * sin(ALPHA) + noise;
	return Y;
}
float KALMAN::calcZ(float t, float noise = 0)
{
	// Computing measurement points e.g. ball trajectory with noise y position
	// See formula in pdf
	float Z = -0.5*G*t*t + V_0 * sin(ALPHA) * t + Z_0 + noise;
	return Z;
}




cv::KalmanFilter KALMAN::positionKF()
{
	cv::KalmanFilter KF(3, 3, 0); // (pos_x, pos_y, pos_z)
	KF.transitionMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0, 
													0, 1, 0,
													0, 0, 1);
	KF.statePost.at<float>(0) = KALMAN::calcX(T_0);
	KF.statePost.at<float>(1) = KALMAN::calcY(T_0);
	KF.statePost.at<float>(2) = KALMAN::calcY(T_0);

	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, cv::Scalar::all(PROCESS_NOISE));
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(MEASUREMENT_NOISE));

	return KF;
}

cv::KalmanFilter KALMAN::velocityKF()
{
	cv::KalmanFilter KF(6, 3, 0); // (pos_x, pos_y, pos_z, vel_x, vel_y, vel_z)
	KF.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, DT, 0, 0, // pos_x
													0, 1, 0, 0, DT, 0, // pos_y
													0, 0, 1, 0, 0, DT, // pos_z
													0, 0, 0, 1, 0, 0, // vel_x
													0, 0, 0, 0, 1, 0, // vel_y
 													0, 0, 0, 0, 0, 1); // vel_z
	// We calculate the velocity based on the corrected position points, however it could be done with the predicted state?
	
	KF.statePost.at<float>(0) = KALMAN::calcX(T_0);
	KF.statePost.at<float>(1) = KALMAN::calcY(T_0);
	KF.statePost.at<float>(2) = KALMAN::calcZ(T_0);

	setIdentity(KF.measurementMatrix); // H matrix
	setIdentity(KF.processNoiseCov, cv::Scalar::all(PROCESS_NOISE));
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(MEASUREMENT_NOISE));

	// Define transitionMatrix which includes position and velocity

	return KF;
}

cv::KalmanFilter KALMAN::accelerationKF()
{
	cv::KalmanFilter KF(9, 3, 0); // (pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, a_x, a_y, a_z)
	KF.transitionMatrix = (cv::Mat_<float>(9, 9) << 1, 0, 0, DT, 0, 0, 0.5*DT*DT, 0, 0, // pos_x
													0, 1, 0, 0, DT, 0, 0, 0.5*DT*DT, 0, // pos_y
													0, 0, 1, 0, 0, DT, 0, 0, 0.5*DT*DT, // pos_z
													0, 0, 0, 1, 0, 0, DT, 0, 0, // vel_x
													0, 0, 0, 0, 1, 0, 0, DT, 0, // vel_y
													0, 0, 0, 0, 0, 1, 0, 0, DT, // vel_z
													0, 0, 0, 0, 0, 0, 1, 0, 0, // a_x
													0, 0, 0, 0, 0, 0, 0, 1, 0, // a_y
													0, 0, 0, 0, 0, 0, 0, 0, 1); // a_z

	KF.statePost.at<float>(0) = KALMAN::calcX(T_0);
	KF.statePost.at<float>(1) = KALMAN::calcY(T_0);
	KF.statePost.at<float>(2) = KALMAN::calcZ(T_0);

	setIdentity(KF.measurementMatrix); // H matrix
	setIdentity(KF.processNoiseCov, cv::Scalar::all(PROCESS_NOISE));
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(MEASUREMENT_NOISE));

	// Define transitionMatrix which includes position, velocity and acceleration

	return KF;
}

cv::Point3_<float> KALMAN::predict(cv::KalmanFilter &KF)
{
	// Do a prediction and return as cv::Point
	
	cv::Mat prediction = KF.predict();
	//cout << "KalmanFilter::Predict" << endl;
	cv::Point3_<float> predicted_pt(prediction.at<float>(0),prediction.at<float>(1), prediction.at<float>(2));
	//cv::Point predicted_pt(0,0);
	return predicted_pt;

}

cv::Point3_<float> KALMAN::correct(cv::KalmanFilter &KF, cv::Mat measurement)
{
	// Do a correction and return as cv::Point
	//cout << "KalmanFilter::Correct" << endl;
	cv::Mat update = KF.correct(measurement);
	cv::Point3_<float> corrected_pt(update.at<float>(0),update.at<float>(1), update.at<float>(2));
	return corrected_pt;
}

void KALMAN::skipCorrect(cv::KalmanFilter &KF)
{
	// Skip correction as described in pdf
	KF.statePost = KF.statePre;
	KF.errorCovPost = KF.errorCovPre; 
}

float KALMAN::L2error(cv::Point pt1, cv::Point pt2)
{
	// Return the L2 error between two points
	float d = sqrt(pow((pt1.x - pt2.x),2) + pow((pt1.y - pt2.y),2));
	return d;
}

float KALMAN::L3error(cv::Point3_<float> pt1, cv::Point3_<float> pt2)
{
	// Return the L2 error between two points
	float d = sqrt(pow((pt1.x - pt2.x),2) + pow((pt1.y - pt2.y),2) + pow((pt1.z - pt2.z),2));
	return d;
}

cv::Point3_<float> KALMAN::Kalman_filter_3D(cv::Mat measPt){
	
	if (!is_initialized){

		cv::KalmanFilter KF = KALMAN::positionKF();
		cv::KalmanFilter KF2 = KALMAN::velocityKF();
		KF3 = KALMAN::accelerationKF();

		std::vector<cv::Point3_ <float> > ground_truth;
		std::vector<cv::Point3_ <float> > measurementPoints;
		std::vector<cv::Point3_ <float> > KF3D_Points;
		std::vector<float> error_points;
		is_initialized = true;
	}

	 // I am not sure to call this here.. it should be done only one time but with init dont work
	cv::Point3_<float> kf3_predicted = KALMAN::predict(KF3);
	cv::Point3_<float> kf3 = kf3_predicted;
	//KF3D_Points.push_back(kf3_predicted);
	//cout << "Ball Center prediction : " << kf3_predicted << endl;
	//cout << "Ball Center Measurement : " << measPt << endl;
	float test = measPt.at<float>(0,0);
	//measurementPoints.push_back(measPt)
	if (isnan(test))
	{
		//cout << "Correction skipped" << endl << endl << endl;
		skipCorrect(KF3);
	}
	else 
	{
		cv::Point3_<float> kf3_corrected = KALMAN::correct(KF3, measPt);
		kf3 = kf3_corrected;
	//cout << "Ball Center Correction : " << kf3_corrected << endl;

	}
	

	//cout << "Prediction error : " << KALMAN::L3error(measPt, kf3_predicted) << endl;
	//cout << "Corrected error : " << KALMAN::L3error(measPt, kf3_corrected) << endl;
	//error_points.push_back(KALMAN::L3error(measPt, kf3));
	return kf3;
}

// Global variables

float x, y, z;
ros::Publisher pub;



// Callback function
void KALMAN::ball_location_kf_callback(const geometry_msgs::PointStamped::ConstPtr &msg)	
{
	x = msg->point.x;
	y = msg->point.y;
	z = msg->point.z;
	cv::Mat_<float> meas(3, 1);
	meas(0) = x;
	meas(1) = y;
	meas(2) = z;
	;
	// call KF
	//cout << "Ball in position:   (" << x << ", " << y << ", " << z << ")" << endl;

	// // Create timer to get runtime 
	//long countdown = 5000; // miliseconds
	//rw::common::Timer timer = rw::common::Timer(countdown);
	//timer.resetAndPause();
	

	cv::Point3_<float> kf_3d = KALMAN::Kalman_filter_3D(meas); // init KF

	//timer.resetAndResume(); // Start timer

	//double time_past = timer.getTime();
	//timer.resetAndPause();

	
	//cout << "Kalman filter checked in:  " << time_past << " seconds." << endl;
	KF_location_3D.header.stamp = msg->header.stamp;
	KF_location_3D.point.x = kf_3d.x;
	KF_location_3D.point.y = kf_3d.y;
	KF_location_3D.point.z = kf_3d.z;

	pub.publish(KALMAN::KF_location_3D);
	cout << "KF_x = " << kf_3d.x << ",  KF_y = " << kf_3d.y << ",  KF_z = " << kf_3d.z << endl;

}// ball_location_callback


