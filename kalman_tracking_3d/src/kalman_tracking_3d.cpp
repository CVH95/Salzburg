/**
 * @file   kalman_tracking_3d.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "kalman_tracking_3d/kalman_tracking_3d.h"

namespace kalman_tracking_3d
{
KalmanTacking3d::KalmanTacking3d(std::string type) : kf_type_(type)
{
  if (kf_type_ == "position")
  {
    kf_ = positionKF();
  }
  else if (kf_type_ == "velocity")
  {
    kf_ = velocityKF();
  }
  else
  {
    kf_ = accelerationKF();
  }
}

KalmanTacking3d::~KalmanTacking3d()
{
}

float KalmanTacking3d::calcU(float t, float noise = 0)
{
  // Computing measurement points e.g. ball trajectory with noise x position
  // See formula in pdf
  float U = V_0 * cos(ALPHA) * t + U_0;
  return U;
}

float KalmanTacking3d::calcX(float t, float noise = 0)
{
  // Computing measurement points e.g. ball trajectory with noise x position
  // See formula in pdf
  float X = calcU(t) * cos(ALPHA) + noise;
  return X;
}

float KalmanTacking3d::calcY(float t, float noise = 0)
{
  // Computing measurement points e.g. ball trajectory with noise y position
  // See formula in pdf
  float Y = calcU(t) * sin(ALPHA) + noise;
  return Y;
}

float KalmanTacking3d::calcZ(float t, float noise = 0)
{
  // Computing measurement points e.g. ball trajectory with noise y position
  // See formula in pdf
  float Z = -0.5 * G * t * t + V_0 * sin(ALPHA) * t + Z_0 + noise;
  return Z;
}

cv::KalmanFilter KalmanTacking3d::positionKF()
{
  cv::KalmanFilter kf(3, 3, 0);  // (pos_x, pos_y, pos_z)
  kf.transitionMatrix = (cv::Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  kf.statePost.at<float>(0) = calcX(T_0);
  kf.statePost.at<float>(1) = calcY(T_0);
  kf.statePost.at<float>(2) = calcZ(T_0);

  setIdentity(kf.measurementMatrix);
  setIdentity(kf.processNoiseCov, cv::Scalar::all(PROCESS_NOISE));
  setIdentity(kf.measurementNoiseCov, cv::Scalar::all(MEASUREMENT_NOISE));

  return kf;
}

cv::KalmanFilter KalmanTacking3d::velocityKF()
{
  cv::KalmanFilter kf(6, 3, 0);                                       // (pos_x, pos_y, pos_z, vel_x, vel_y, vel_z)
  kf.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, DT, 0, 0,  // pos_x
                         0, 1, 0, 0, DT, 0,                           // pos_y
                         0, 0, 1, 0, 0, DT,                           // pos_z
                         0, 0, 0, 1, 0, 0,                            // vel_x
                         0, 0, 0, 0, 1, 0,                            // vel_y
                         0, 0, 0, 0, 0, 1);                           // vel_z

  kf.statePost.at<float>(0) = calcX(T_0);
  kf.statePost.at<float>(1) = calcY(T_0);
  kf.statePost.at<float>(2) = calcZ(T_0);

  setIdentity(kf.measurementMatrix);  // H matrix
  setIdentity(kf.processNoiseCov, cv::Scalar::all(PROCESS_NOISE));
  setIdentity(kf.measurementNoiseCov, cv::Scalar::all(MEASUREMENT_NOISE));

  return kf;
}

cv::KalmanFilter KalmanTacking3d::accelerationKF()
{
  cv::KalmanFilter kf(9, 3, 0);  // (pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, a_x, a_y, a_z)
  kf.transitionMatrix = (cv::Mat_<float>(9, 9) << 1, 0, 0, DT, 0, 0, 0.5 * DT * DT, 0, 0,  // pos_x
                         0, 1, 0, 0, DT, 0, 0, 0.5 * DT * DT, 0,                           // pos_y
                         0, 0, 1, 0, 0, DT, 0, 0, 0.5 * DT * DT,                           // pos_z
                         0, 0, 0, 1, 0, 0, DT, 0, 0,                                       // vel_x
                         0, 0, 0, 0, 1, 0, 0, DT, 0,                                       // vel_y
                         0, 0, 0, 0, 0, 1, 0, 0, DT,                                       // vel_z
                         0, 0, 0, 0, 0, 0, 1, 0, 0,                                        // a_x
                         0, 0, 0, 0, 0, 0, 0, 1, 0,                                        // a_y
                         0, 0, 0, 0, 0, 0, 0, 0, 1);                                       // a_z

  kf.statePost.at<float>(0) = calcX(T_0);
  kf.statePost.at<float>(1) = calcY(T_0);
  kf.statePost.at<float>(2) = calcZ(T_0);

  setIdentity(kf.measurementMatrix);  // H matrix
  setIdentity(kf.processNoiseCov, cv::Scalar::all(PROCESS_NOISE));
  setIdentity(kf.measurementNoiseCov, cv::Scalar::all(MEASUREMENT_NOISE));
  return kf;
}

Eigen::Vector3f KalmanTacking3d::predict(cv::KalmanFilter& kf)
{
  // Do a prediction and return as cv::Point
  cv::Mat m = kf.predict();
  cv::Point3_<float> pt(m.at<float>(0), m.at<float>(1), m.at<float>(2));

  Eigen::Vector3f prediction(pt.x, pt.y, pt.z);
  return prediction;
}

Eigen::Vector3f KalmanTacking3d::correct(cv::KalmanFilter& kf, cv::Mat measurement)
{
  // Do a correction and return as cv::Point
  cv::Mat update = kf.correct(measurement);
  cv::Point3_<float> pt(update.at<float>(0), update.at<float>(1), update.at<float>(2));

  Eigen::Vector3f correction(pt.x, pt.y, pt.z);
  return correction;
}

void KalmanTacking3d::skipCorrect(cv::KalmanFilter& kf)
{
  // Skip correction as described in pdf
  kf.statePost = kf.statePre;
  kf.errorCovPost = kf.errorCovPre;
}

float KalmanTacking3d::L2Error(cv::Point pt1, cv::Point pt2)
{
  // Return the L2 error between two points
  float d = sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
  return d;
}

float KalmanTacking3d::L3Error(Eigen::Vector3f pt1, Eigen::Vector3f pt2)
{
  // Return the L2 error between two points
  float d = sqrt(pow((pt1.x() - pt2.x()), 2) + pow((pt1.y() - pt2.y()), 2) + pow((pt1.z() - pt2.z()), 2));
  return d;
}

Eigen::Vector3f KalmanTacking3d::kalmanFilter3d(cv::Mat meas_pt, cv::KalmanFilter& kf)
{
  Eigen::Vector3f kf_vec = predict(kf);

  float test = meas_pt.at<float>(0, 0);
  if (isnan(test))
  {
    skipCorrect(kf);
  }
  else
  {
    kf_vec = correct(kf, meas_pt);
  }

  return kf_vec;
}

}  // namespace kalman_tracking_3d