/**
 * @file   kinect_detection.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "perception_avd/kinect_detection.h"

namespace perception_avd
{
KinectDetection::KinectDetection(ros::NodeHandle node_handle) : nh_(node_handle)
{
  if (!readParams())
  {
    ROS_ERROR("Error reading from paramter server!");
    exit(1);
  }

  ROS_INFO("--- KINECT DETECTION ---");
  ROS_INFO(" * Publishing on topic: %s", pose_topic_.c_str());
  ROS_INFO(" * Input depth image: %s", depth_topic_.c_str());
  ROS_INFO(" * Input rgb image: %s", image_topic_.c_str());
  ROS_INFO(" * Calibration file: %s", cam_calib_.c_str());
  if (publish_result_image_)
  {
    ROS_INFO(" * Publishing images on: %s", result_image_topic_.c_str());
  }

  kalman_ = new kalman_tracking_3d::KalmanTacking3d("velocity");
  kf_ = kalman_->velocityKF();

  setCameraData();
}

KinectDetection::~KinectDetection()
{
}

bool KinectDetection::readParams()
{
  bool success = true;
  success = success && ros::param::get("pose_topic", pose_topic_);
  success = success && ros::param::get("depth_image_topic", depth_topic_);
  success = success && ros::param::get("rgb_image_topic", image_topic_);
  success = success && ros::param::get("publish_result_image", publish_result_image_);
  success = success && ros::param::get("result_image_topic", result_image_topic_);
  success = success && ros::param::get("ball_radius", ball_radius_);
  success = success && ros::param::get("camera_calibration", cam_calib_);
  return success;
}

void KinectDetection::initPublishers()
{
  pose_pub_ = nh_.advertise<rovi2_msgs::points3d>(pose_topic_, 1);
  if (publish_result_image_)
  {
    result_pub_ = nh_.advertise<sensor_msgs::Image>(result_image_topic_, 1);
  }
}

// Load camera paramters
void KinectDetection::setCameraData()
{
  // Obtain extrinsics
  cv::FileStorage fs(cam_calib_, cv::FileStorage::READ);

  fs["camera_frame"] >> camera_frame_;
  fs["camera_matrix"] >> k_instrinsics_left_;
  fs["distortion_coefficients"] >> dist_coeffs_left_;
  fs.release();

  focal_length_ = k_instrinsics_left_.at<double>(0, 0);
  camera_center_.x = k_instrinsics_left_.at<double>(0, 2);
  camera_center_.y = k_instrinsics_left_.at<double>(1, 2);

  ROS_INFO("Kinect data: ");
  ROS_INFO(" * Ball radius: %f", ball_radius_);
  ROS_INFO(" * Focal length: %f", focal_length_);
  ROS_INFO(" * Image Center: (%f, %f)", camera_center_.x, camera_center_.y);
}

void KinectDetection::synchronizedCallback(const sensor_msgs::ImageConstPtr& rgb,
                                           const sensor_msgs::ImageConstPtr& depth)
{
  cv_bridge::CvImagePtr depth_ptr, rgb_ptr;
  try
  {
    depth_ptr = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat depth_img = depth_ptr->image;

    rgb_ptr = cv_bridge::toCvCopy(*rgb, sensor_msgs::image_encodings::BGR8);
    cv::Mat rgb_img = rgb_ptr->image;

    rovi2_msgs::points2d vector_of_points;
    rovi2_msgs::boundingBoxes bb_corners;

    findCenters(rgb_img, vector_of_points, bb_corners);

    if (publish_result_image_)
    {
      // Publish
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_img).toImageMsg();
      result_pub_.publish(msg);
    }

    rovi2_msgs::points3d obstacle_track;
    for (int i = 0; i < vector_of_points.points.size(); i++)
    {
      rovi2_msgs::point2d point_i = vector_of_points.points[i];

      float final_z = (float)depth_img.at<float>(static_cast<int>(point_i.y), static_cast<int>(point_i.x));
      float final_x = (float)(point_i.x - camera_center_.x) * final_z / focal_length_;
      float final_y = (float)(point_i.y - camera_center_.y) * final_z / focal_length_;

      ROS_INFO("Detected obstacle %i at (%f, %f, %f)", i, final_x, final_y, final_z);

      // Kalman filte
      cv::Mat_<float> measurement(3, 1);
      measurement(0) = final_x;
      measurement(1) = final_y;
      measurement(2) = final_z;

      Eigen::Vector3f track = kalman_->kalmanFilter3d(measurement, kf_);
      rovi2_msgs::point3d k;
      k.x = track.x();
      k.y = track.y();
      k.z = track.z();
      k.object_id = 0;

      broadcastDetectedTf(k, std::to_string(i), camera_frame_);
      obstacle_track.points.push_back(k);
      ROS_INFO("Tracked obstacle %i at (%f, %f, %f)", i, k.x, k.y, k.z);
    }

    obstacle_track.header.stamp = ros::Time::now();
    obstacle_track.header.frame_id = camera_frame_;
    pose_pub_.publish(obstacle_track);

    ROS_INFO("---------------");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Exception:  %s", e.what());
    return;
  }
}

}  // namespace perception_avd