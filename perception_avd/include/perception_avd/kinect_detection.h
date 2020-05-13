/**
 * @file   kinect_detection.h
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief PCL detection: Color-based segmentation + object clustering + pose estimation of the cluster.
 * @brief Publishes image points corresponding to object centers.
 */

#ifndef KINECT_DETECTION
#define KINECT_DETECTION

#include <string>
#include <vector>
#include <iostream>

#include <ros/package.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>

#include "perception_avd/common_perception.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include "kalman_tracking_3d/kalman_tracking_3d.h"

namespace perception_avd
{
class KinectDetection
{
private:
  ros::NodeHandle nh_;

  // Pubs & Subs
  ros::Publisher pose_pub_, result_pub_;

  // Params
  std::string pose_topic_, cam_calib_, result_image_topic_;
  bool publish_result_image_;
  double ball_radius_;

  // Camera data
  cv::Mat k_instrinsics_left_, dist_coeffs_left_;
  cv::Point2d camera_center_;
  double focal_length_;
  std::string camera_frame_;

  kalman_tracking_3d::KalmanTacking3d* kalman_;
  cv::KalmanFilter kf_;

public:
  KinectDetection(ros::NodeHandle node_handle);
  ~KinectDetection();

  bool readParams();
  void initPublishers();
  void setCameraData();

  void synchronizedCallback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth);

  // Public params
  std::string image_topic_, depth_topic_;
};
}  // namespace perception_avd

#endif  // KINECT_DETECTION