/**
 * @file   common_perception.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "perception_avd/common_perception.h"

namespace perception_avd
{
// Red ball detection
void findCenters(cv::Mat& image, rovi2_msgs::points2d& vector_of_points, rovi2_msgs::boundingBoxes& bb_corners)
{
  cv::Mat hsv;
  cv::GaussianBlur(image, hsv, cv::Size(3, 3), 2, 2);
  cv::cvtColor(hsv, hsv, CV_BGR2HSV);

  // Filter in HSV space.
  cv::Mat hue1, hue2;
  cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), hue1);
  cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), hue2);  // Hue range for red colors: 160-179.

  cv::Mat red_ball;
  cv::addWeighted(hue1, 1.0, hue2, 1.0, 0.0, red_ball);

  int m_size = 2;
  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(m_size + 1, m_size + 1), cv::Point(m_size, m_size));
  morphologyEx(red_ball, red_ball, cv::MORPH_OPEN, element, cv::Point(-1, -1), 6);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(red_ball, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  std::vector<std::vector<cv::Point>> contours_poly(contours.size());
  std::vector<cv::Rect> bound_rect(contours.size());
  std::vector<cv::Point2f> center(contours.size());
  std::vector<float> radius(contours.size());
  std::vector<std::vector<cv::Point2f>> cv_bb_corners;

  for (int i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(contours[i], contours_poly[i], 5, true);
    bound_rect[i] = cv::boundingRect(contours_poly[i]);
    cv::minEnclosingCircle(contours_poly[i], center[i], radius[i]);
  }

  if (center.size() == 0)
  {
    return;
  }

  for (int i = 0; i < center.size(); i++)
  {
    // Get rovi2_msgs::points2d ball centers
    rovi2_msgs::point2d coordinates;

    coordinates.object_id = i;
    coordinates.x = center[i].x;
    coordinates.y = center[i].y;

    vector_of_points.points.push_back(coordinates);  // output centers

    // Get rovi2_msgs::boundingBox boxes
    rovi2_msgs::boundingBox corners = getBoxFromRect(bound_rect[i]);
    std::vector<cv::Point2f> cv_corners = getCornersFromRect(bound_rect[i]);
    bb_corners.box.push_back(corners);  // Output
    cv_bb_corners.push_back(cv_corners);
  }

  // Draw
  cv::Mat box = cv::Mat::zeros(red_ball.size(), CV_8UC3);
  for (int i = 0; i < contours.size(); i++)
  {
    cv::drawContours(box, contours, -1, cv::Scalar(0, 0, 255), 1);
    cv::rectangle(box, bound_rect[i], cv::Scalar(0, 255, 255), 3, 8, 0);
    cv::circle(box, center[i], radius[i], cv::Scalar(0, 0, 255), -1, 8, 0);
    std::vector<cv::Point2f> bb_i = cv_bb_corners[i];
    for (int j = 0; j < bb_i.size(); j++)
    {
      cv::circle(box, bb_i[j], 10, cv::Scalar(255, 255, 255), -1, 8, 0);
    }
  }

  image = box;  // Output image
}

// Create bounding box array for cv::Mat drawing
std::vector<cv::Point2f> getCornersFromRect(cv::Rect rectangle)
{
  std::vector<cv::Point2f> corners;
  cv::Point2f top_left(rectangle.x, rectangle.y);
  cv::Point2f top_right(rectangle.x + rectangle.width, rectangle.y);
  cv::Point2f bottom_left(rectangle.x, rectangle.y + rectangle.height);
  cv::Point2f bottom_right(rectangle.x + rectangle.width, rectangle.y + rectangle.height);

  corners.push_back(top_left);
  corners.push_back(top_right);
  corners.push_back(bottom_left);
  corners.push_back(bottom_right);

  return corners;
}

// Create bounding box msg
rovi2_msgs::boundingBox getBoxFromRect(cv::Rect rectangle)
{
  rovi2_msgs::boundingBox corners;
  corners.top_left.x = rectangle.x;
  corners.top_left.y = rectangle.y;
  corners.top_right.x = rectangle.x + rectangle.width;
  corners.top_right.y = rectangle.y;
  corners.bottom_left.x = rectangle.x;
  corners.bottom_left.y = rectangle.y + rectangle.height;
  corners.bottom_right.x = rectangle.x + rectangle.width;
  corners.bottom_right.y = rectangle.y + rectangle.height;

  return corners;
}

// Get corner array from bounding box msg
std::vector<cv::Point2f> getCornersFromBox(rovi2_msgs::boundingBox bb)
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

// std::out opencv matrix
void printMatrix(cv::Mat matrix)
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

// Calculate euclidean distance between two rovi2_msgs::point2d points
float euclideanDistance(rovi2_msgs::point2d left_point, rovi2_msgs::point2d right_point)
{
  float difference = (right_point.x - left_point.x) * (right_point.x - left_point.x) +
                     (right_point.y - left_point.y) * (right_point.y - left_point.y);
  return sqrt(difference);
}

// Send TF of detected pose
void broadcastDetectedTf(rovi2_msgs::point3d p, std::string id, std::string camera_frame)
{
  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = camera_frame;
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
}  // namespace perception_avd