/**
 * @file   stereopsis.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "perception_avd/stereopsis.h"

namespace perception_avd
{
Stereopsis::Stereopsis(ros::NodeHandle node_handle) : nh_(node_handle)
{
  if (!readParams())
  {
    ROS_ERROR("Error reading from paramter server!");
    exit(1);
  }

  if (monocular_)
  {
    ROS_INFO("Monocular stereopsis node initialized!");
    ROS_INFO(" * Publishing on topic: %s", pub_topic_.c_str());
    ROS_INFO(" * Diameter: %f", diameter_);
    ROS_INFO(" * Bounding box topic: %s", subscribe_topic_.c_str());
  }
  else
  {
    ROS_INFO("Stereopsis node initialized!");
    ROS_INFO(" * Publishing on topic: %s", pub_topic_.c_str());
    ROS_INFO(" * Stereo baseline: %f", baseline_);
  }

  ros::Duration(0.5).sleep();

  kalman_ = new kalman_tracking_3d::KalmanTacking3d("velocity");
  kf_ = kalman_->velocityKF();

  setProjectionMatrices();

  ros::Duration(1).sleep();
}

Stereopsis::~Stereopsis()
{
}

bool Stereopsis::readParams()
{
  bool success = true;
  success = success && ros::param::get("pub_topic", pub_topic_);
  success = success && ros::param::get("left_camera_calibration", left_cam_calib_);
  success = success && ros::param::get("right_camera_calibration", right_cam_calib_);
  success = success && ros::param::get("stereo_baseline", baseline_);
  success = success && ros::param::get("diameter", diameter_);
  success = success && ros::param::get("monocular", monocular_);
  success = success && ros::param::get("monocular_topic", subscribe_topic_);
  return success;
}

void Stereopsis::initPublishers()
{
  ws_monitoring_pub_ = nh_.advertise<rovi2_msgs::points3d>(pub_topic_, 1);
}

void Stereopsis::initSubscribers()
{
  corners_sub_ = nh_.subscribe(subscribe_topic_, 1, &Stereopsis::cornersCallback, this);
}

// Load camera paramters
void Stereopsis::setProjectionMatrices()
{
  // Obtain extrinsics
  cv::FileStorage left_fs(left_cam_calib_, cv::FileStorage::READ);
  cv::FileStorage right_fs(right_cam_calib_, cv::FileStorage::READ);

  left_fs["camera_frame"] >> left_camera_frame_;
  left_fs["camera_matrix"] >> k_instrinsics_left_;
  left_fs["distortion_coefficients"] >> dist_coeffs_left_;
  left_fs["rectification_matrix"] >> r_rectification_left_;
  left_fs["projection_matrix"] >> projection_mat_left_;

  right_fs["camera_frame"] >> right_camera_frame_;
  right_fs["camera_matrix"] >> k_intrinsics_right_;
  right_fs["distortion_coefficients"] >> dist_coeffs_right_;
  right_fs["rectification_matrix"] >> r_rectification_right_;
  right_fs["projection_matrix"] >> projection_mat_right_;

  left_fs.release();
  right_fs.release();

  std::cout << "Camera frame: " << left_camera_frame_ << std::endl;

  std::cout << std::endl << "Camera matrices" << std::endl;
  printMatrix(k_instrinsics_left_);
  printMatrix(k_intrinsics_right_);

  std::cout << std::endl << "R matrices" << std::endl;
  printMatrix(r_rectification_left_);
  printMatrix(r_rectification_right_);

  cv::Mat t_left(3, 1, CV_64F);
  cv::Mat t_right(3, 1, CV_64F);

  t_left.at<double>(0, 0) =
      -(projection_mat_left_.at<double>(0, 3) / projection_mat_left_.at<double>(0, 0));  // baseline = Tx() / fx()
  f_ = projection_mat_left_.at<double>(0, 0);
  t_left.at<double>(1, 0) = -(projection_mat_left_.at<double>(1, 3) / projection_mat_left_.at<double>(1, 1));
  t_left.at<double>(2, 0) = 0.0;
  t_right.at<double>(0, 0) = -(projection_mat_right_.at<double>(0, 3) / projection_mat_right_.at<double>(0, 0));
  t_right.at<double>(1, 0) = -(projection_mat_right_.at<double>(1, 3) / projection_mat_right_.at<double>(1, 1));
  t_right.at<double>(2, 0) = 0.0;

  baseline_ = t_right.at<double>(0, 0);

  cv::Mat m_left(3, 4, CV_64F);
  cv::Mat m_right(3, 4, CV_64F);

  cv::hconcat(r_rectification_left_, t_left, m_left);
  cv::hconcat(r_rectification_right_, t_right, m_right);

  std::cout << "Baseline = " << baseline_ << " | f = " << f_ << std::endl;
  std::cout << std::endl << "M 3x4 matrices:" << std::endl;
  printMatrix(m_left);
  printMatrix(m_right);

  // P = K*M
  extrinsic_projection_left_ = k_instrinsics_left_ * m_left;
  extrinsic_projection_right_ = k_intrinsics_right_ * m_right;

  std::cout << std::endl << "Extrinsic projection matrices:" << std::endl;
  printMatrix(extrinsic_projection_left_);
  printMatrix(extrinsic_projection_right_);
}

// Stereo matching based on NN
std::vector<std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d>>
Stereopsis::stereoMatching(rovi2_msgs::points2d left_array, rovi2_msgs::points2d right_array)
{
  // NN Matching (Euclidean Distance)
  std::vector<std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d>> center_pairs;
  for (int i = 0; i < left_array.points.size(); i++)
  {
    rovi2_msgs::point2d left = left_array.points[i];
    std::vector<float> distances;
    for (int j = 0; j < right_array.points.size(); j++)
    {
      float distance_j = euclideanDistance(left, right_array.points[j]);
      distances.push_back(distance_j);
    }
    float min = distances[0];
    int index = 0;
    for (int h = 0; h < distances.size(); h++)
    {
      if (distances[h] < min)
      {
        min = distances[h];
        index = h;
      }
    }

    std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d> pair(left, right_array.points[index]);
    center_pairs.push_back(pair);
    // ROS_INFO("Pair left->(%f, %f) | right->(%f, %f)", left.x, left.y, right_array.objects[index].x,
    // right_array.objects[index].y);
  }
  return center_pairs;
}

// Triangulate (call to OpenCV's triangulatePoints())
rovi2_msgs::points3d
Stereopsis::triangulateObjectsCv(std::vector<std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d>> center_pairs)
{
  rovi2_msgs::points3d locations;

  // Triangulation
  for (int i = 0; i < center_pairs.size(); i++)
  {
    rovi2_msgs::point2d pair_left = std::get<0>(center_pairs[i]);
    rovi2_msgs::point2d pair_right = std::get<1>(center_pairs[i]);

    cv::Mat point_in_3d(1, 1, CV_64FC4);
    cv::Mat left_camera_detection(1, 1, CV_64FC2);
    cv::Mat right_camera_detection(1, 1, CV_64FC2);
    left_camera_detection.at<cv::Vec2d>(0)[0] = pair_left.x;
    left_camera_detection.at<cv::Vec2d>(0)[1] = pair_left.y;
    right_camera_detection.at<cv::Vec2d>(0)[0] = pair_right.x;
    right_camera_detection.at<cv::Vec2d>(0)[1] = pair_right.x;

    cv::triangulatePoints(extrinsic_projection_left_, extrinsic_projection_right_, left_camera_detection,
                          right_camera_detection, point_in_3d);

    rovi2_msgs::point3d p;
    p.x = point_in_3d.at<double>(0, 0) / point_in_3d.at<double>(3, 0);
    p.y = point_in_3d.at<double>(1, 0) / point_in_3d.at<double>(3, 0);
    p.z = point_in_3d.at<double>(2, 0) / point_in_3d.at<double>(3, 0);

    p.object_id = i;

    locations.points.push_back(p);
  }
  locations.header.stamp = ros::Time::now();
  locations.header.frame_id = left_camera_frame_;
  return locations;
}

rovi2_msgs::points3d Stereopsis::triangulateObjectsParallelCam(
    std::vector<std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d>> center_pairs)
{
  rovi2_msgs::points3d locations;

  // Triangulation
  for (int i = 0; i < center_pairs.size(); i++)
  {
    rovi2_msgs::point2d pair_left = std::get<0>(center_pairs[i]);
    rovi2_msgs::point2d pair_right = std::get<1>(center_pairs[i]);

    rovi2_msgs::point3d p;
    p.z = (baseline_ * f_) / (pair_left.x - pair_right.x);
    p.x = pair_left.x * (p.z / f_);
    p.y = pair_left.y * (p.z / f_);
    p.object_id = i;
    locations.points.push_back(p);
  }
  locations.header.stamp = ros::Time::now();
  locations.header.frame_id = left_camera_frame_;
  return locations;
}

void Stereopsis::printMatrix(cv::Mat matrix)
{
  std::cout << "Matrix [" << matrix.rows << "x" << matrix.cols << "] = " << std::endl;
  for (int i = 0; i < matrix.rows; i++)
  {
    for (int j = 0; j < matrix.cols; j++)
    {
      std::cout << matrix.at<double>(i, j) << "  ";
    }
    std::cout << std::endl;
  }
}

float Stereopsis::euclideanDistance(rovi2_msgs::point2d left_point, rovi2_msgs::point2d right_point)
{
  float difference = (right_point.x - left_point.x) * (right_point.x - left_point.x) +
                     (right_point.y - left_point.y) * (right_point.y - left_point.y);
  return sqrt(difference);
}

void Stereopsis::broadcastDetectedTf(rovi2_msgs::point3d p, std::string id)
{
  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = left_camera_frame_;
  ts.child_frame_id = "detection_" + id;

  ts.transform.translation.x = p.x;
  ts.transform.translation.y = p.y;
  ts.transform.translation.z = p.z;
  ts.transform.rotation.x = 0.0;
  ts.transform.rotation.y = 0.0;
  ts.transform.rotation.z = 0.0;
  ts.transform.rotation.w = 1.0;

  br.sendTransform(ts);
}

void Stereopsis::synchronized_triangulation(const rovi2_msgs::points2d::ConstPtr& left_msg,
                                            const rovi2_msgs::points2d::ConstPtr& right_msg)
{
  rovi2_msgs::points2d left_detection = *left_msg;
  rovi2_msgs::points2d right_detection = *right_msg;

  ROS_INFO("---------------");

  if (left_detection.points.empty() || right_detection.points.empty())
  {
    ROS_WARN("No detections received");
    return;
  }

  std::vector<std::tuple<rovi2_msgs::point2d, rovi2_msgs::point2d>> center_pairs =
      stereoMatching(left_detection, right_detection);
  // rovi2_msgs::points3d obstacles = triangulateObjectsCv(center_pairs);
  rovi2_msgs::points3d obstacles = triangulateObjectsParallelCam(center_pairs);

  // Tracking with Kalman filter
  rovi2_msgs::points3d obstacle_track;
  for (int i = 0; i < obstacles.points.size(); i++)
  {
    rovi2_msgs::point3d p = obstacles.points[i];
    ROS_INFO("Detected obstacle at (%f, %f, %f)", p.x, p.y, p.z);

    // Kalman filter
    cv::Mat_<float> measurement(3, 1);
    measurement(0) = p.x;
    measurement(1) = p.y;
    measurement(2) = p.z;

    Eigen::Vector3f track = kalman_->kalmanFilter3d(measurement, kf_);
    rovi2_msgs::point3d k;
    k.x = track.x();
    k.y = track.y();
    k.z = track.z();
    k.object_id = p.object_id;

    broadcastDetectedTf(k, std::to_string(i));
    obstacle_track.points.push_back(k);
    ROS_INFO("Tracked obstacle at (%f, %f, %f)", k.x, k.y, k.z);
  }

  obstacle_track.header.stamp = ros::Time::now();
  obstacle_track.header.frame_id = left_camera_frame_;
  ws_monitoring_pub_.publish(obstacle_track);

  ROS_INFO("---------------");
}

std::vector<cv::Point2f> Stereopsis::getCornersFromBox(rovi2_msgs::boundingBox bb)
{
  std::vector<cv::Point2f> corners;
  cv::Point2f top_left(bb.top_left.x, bb.top_left.y);
  cv::Point2f top_right(bb.top_right.x, bb.top_right.y);
  cv::Point2f bottom_left(bb.bottom_left.x, bb.bottom_left.y);
  cv::Point2f bottom_right(bb.bottom_right.x, bb.bottom_right.y);

  corners.push_back(top_left);
  corners.push_back(top_right);
  corners.push_back(bottom_left);
  corners.push_back(bottom_right);

  return corners;
}

void Stereopsis::cornersCallback(const rovi2_msgs::boundingBoxes& msg)
{
  if (msg.box.size() == 0)
  {
    ROS_WARN("No detections");
    return;
  }

  std::vector<std::vector<cv::Point2f>> cv_corner_array;
  for (int i = 0; i < msg.box.size(); i++)
  {
    ROS_INFO("Detected %i obstacle", msg.box.size());
    std::vector<cv::Point2f> cv_points = getCornersFromBox(msg.box[i]);
    cv_corner_array.push_back(cv_points);
  }

  std::vector<cv::Vec3d> tvecs, rvecs;
  cv::aruco::estimatePoseSingleMarkers(cv_corner_array, diameter_, k_instrinsics_left_, dist_coeffs_left_, rvecs,
                                       tvecs);

  rovi2_msgs::points3d obstacle_track;
  for (int i = 0; i < tvecs.size(); i++)
  {
    ROS_INFO("Detected obstacle %i at (%f, %f, %f)", i, tvecs[i][0], tvecs[i][1], tvecs[i][2]);

    // Kalman filter
    cv::Mat_<float> measurement(3, 1);
    measurement(0) = tvecs[i][0];
    measurement(1) = tvecs[i][1];
    measurement(2) = tvecs[i][2];

    Eigen::Vector3f track = kalman_->kalmanFilter3d(measurement, kf_);

    rovi2_msgs::point3d k;
    k.x = track.x();
    k.y = track.y();
    k.z = track.z();
    k.object_id = msg.box[i].object_id;

    broadcastDetectedTf(k, std::to_string(i));
    obstacle_track.points.push_back(k);

    ROS_INFO("Tracked obstacle %i at (%f, %f, %f)", i, k.x, k.y, k.z);
  }

  obstacle_track.header.stamp = ros::Time::now();
  obstacle_track.header.frame_id = left_camera_frame_;
  ws_monitoring_pub_.publish(obstacle_track);
  ROS_INFO("---------------");
}

void Stereopsis::freeMemory()
{
  delete kalman_;
}

}  // namespace perception_avd