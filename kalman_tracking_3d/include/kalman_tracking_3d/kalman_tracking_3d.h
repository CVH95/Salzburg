/**
 * @file   kalman_tracking_3d.h
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#pragma once

#ifndef KALMAN_TRACKIN_3D
#define KALMAN_TRACKIN_3D

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

#include <Eigen/Geometry>

#define X_0 0    // start position x
#define Y_0 0    // start position y
#define V_0 100  // start velocity
#define Z_0 0
#define ALPHA 1  // start angle
#define T_0 0    // initial time
#define DT 0.15  // timestep
#define U_0 0

#define X_NOISE 1
#define Y_NOISE 0
#define Z_NOISE 2
#define PROCESS_NOISE 0.01        //(Q_)
#define MEASUREMENT_NOISE 1       //(R_)
#define ERROR_COV_POST_NOISE 0.1  //(P_)
#define G 9.81

namespace kalman_tracking_3d
{
class KalmanTacking3d
{
private:
  std::string kf_type_;
  cv::KalmanFilter kf_;

public:
  KalmanTacking3d(std::string type);
  ~KalmanTacking3d();

  // Methods
  float calcU(float t, float noise);
  float calcX(float t, float noise);
  float calcY(float t, float noise);
  float calcZ(float t, float noise);

  cv::KalmanFilter positionKF();
  cv::KalmanFilter velocityKF();
  cv::KalmanFilter accelerationKF();
  Eigen::Vector3f predict(cv::KalmanFilter& kf);
  Eigen::Vector3f correct(cv::KalmanFilter& kf, cv::Mat measurement);
  void skipCorrect(cv::KalmanFilter& kf);
  float L2Error(cv::Point pt1, cv::Point pt2);
  float L3Error(Eigen::Vector3f pt1, Eigen::Vector3f pt2);
  Eigen::Vector3f kalmanFilter3d(cv::Mat meas_pt, cv::KalmanFilter& kf);
};
}  // namespace kalman_tracking_3d

#endif  // KALMAN_TRACKIN_3D
