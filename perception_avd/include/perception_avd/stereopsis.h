/**
 * @file   stereopsis.h
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#ifndef STEREOPSIS_H
#define STEREOPSIS_H

#include <vector>
#include <string>
#include <iostream>
#include <tuple>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rovi2_msgs/point2d.h"
#include "rovi2_msgs/point3d.h"
#include "rovi2_msgs/points2d.h"
#include "rovi2_msgs/points3d.h"
#include "rovi2_msgs/boundingBox.h"
#include "rovi2_msgs/boundingBoxes.h"
#include "geometry_msgs/TransformStamped.h"

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include "kalman_tracking_3d/kalman_tracking_3d.h"

namespace perception_avd
{
class Stereopsis
{
private:
  ros::NodeHandle nh_;

  // Pubs & Subs
  ros::Publisher ws_monitoring_pub_;
  ros::Subscriber corners_sub_;

  // Detection msgs
  rovi2_msgs::points2d left_detection_;
  rovi2_msgs::point2d right_detection_;

  // Private params
  std::string pub_topic_, subscribe_topic_;
  std::string left_cam_calib_, right_cam_calib_;
  bool monocular_;
  double baseline_, f_, diameter_;

  // Stereo vision parameters
  std::string left_camera_frame_, right_camera_frame_;
  cv::Mat k_instrinsics_left_, k_intrinsics_right_;       // Intrinsics
  cv::Mat dist_coeffs_left_, dist_coeffs_right_;          // Distortion
  cv::Mat r_rectification_left_, r_rectification_right_;  // Rectivication
  cv::Mat projection_mat_left_, projection_mat_right_;    // Projective

  cv::Mat extrinsic_projection_left_, extrinsic_projection_right_;  // Final stereo projection matrices

  kalman_tracking_3d::KalmanTacking3d* kalman_;
  cv::KalmanFilter kf_;

  // Monocular triangulation
  std::vector<cv::Point2f> getCornersFromBox(rovi2_msgs::boundingBox bb);
  void cornersCallback(const rovi2_msgs::boundingBoxes& msg);

public:
  Stereopsis(ros::NodeHandle node_handle);
  ~Stereopsis();

  // Pubs & Subs
  void initPublishers();
  void initSubscribers();
  bool readParams();

  // Triangulation
  void setProjectionMatrices();
  std::vector<std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d> > stereoMatching(rovi2_msgs::points2d left_array,
                                                                                    rovi2_msgs::points2d right_array);
  rovi2_msgs::points3d
  triangulateObjectsCv(std::vector<std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d> > center_pairs);
  rovi2_msgs::points3d
  triangulateObjectsParallelCam(std::vector<std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d> > center_pairs);

  // Other methods
  void broadcastDetectedTf(rovi2_msgs::point3d p, std::string id);
  void printMatrix(cv::Mat matrix);
  float euclideanDistance(rovi2_msgs::point2d left_point, rovi2_msgs::point2d right_point);

  // Callbacks
  void synchronized_triangulation(const rovi2_msgs::points2d::ConstPtr& left_msg,
                                  const rovi2_msgs::points2d::ConstPtr& right_msg);

  void freeMemory();
};
}  // namespace perception_avd

#endif  // STEREOPSIS_H
