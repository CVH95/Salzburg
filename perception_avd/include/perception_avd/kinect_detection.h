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

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/PointCloud2.h"
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
class KinectDetection
{
private:
  ros::NodeHandle nh_;

  // Pubs & Subs
  ros::Publisher result_pub_, pose_pub_;
  ros::Subscriber sub_;

  // Params
  std::string result_topic_, subscribe_topic_, pose_topic_;
  std::string camera_frame_;
  bool publish_pcl_;

  kalman_tracking_3d::KalmanTacking3d* kalman_;
  cv::KalmanFilter kf_;

  // Callback
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  // PCL processing
  pcl::PointCloud<pcl::PointXYZRGB> colorSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
  // void objectClustering(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
  Eigen::Vector3d estimateClusterPose(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

public:
  KinectDetection(ros::NodeHandle node_handle);
  ~KinectDetection();

  bool readParams();
  void initPublishers();
  void initSubscribers();

  void broadcastDetectedTf(rovi2_msgs::point3d p, std::string id);
};
}  // namespace perception_avd

#endif  // KINECT_DETECTION