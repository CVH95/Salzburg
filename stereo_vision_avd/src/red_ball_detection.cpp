/**
 * @file   red_ball_detection.cpp
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "stereo_vision_avd/red_ball_detection.h"

namespace stereo_vision_avd
{
RedBallDetection::RedBallDetection(ros::NodeHandle node_handle) : nh_(node_handle)
{
  if (!readParams())
  {
    ROS_ERROR("Could not load params from paramater server!");
    exit(1);
  }

  ROS_INFO("Publishers:");
  ROS_INFO(" * Detections: %s", points_topic_.c_str());
  if (publish_result_image_)
  {
    ROS_INFO(" * Images: %s", output_topic_.c_str());
  }
}

RedBallDetection::~RedBallDetection()
{
}

bool RedBallDetection::readParams()
{
  bool success = true;
  success = success && ros::param::get("image_topic", subscribe_topic_);
  success = success && ros::param::get("points_topic", points_topic_);
  success = success && ros::param::get("output_topic", output_topic_);
  success = success && ros::param::get("publish_images", publish_result_image_);
  return success;
}

void RedBallDetection::initSubscribers()
{
  image_sub_ = nh_.subscribe(subscribe_topic_, 1, &RedBallDetection::imageCallback, this);
}

void RedBallDetection::initPublishers()
{
  points_pub_ = nh_.advertise<rovi2_msgs::points2d>(points_topic_, 1);
  if (publish_result_image_)
  {
    output_pub_ = nh_.advertise<sensor_msgs::Image>(output_topic_, 1);
  }
}

void RedBallDetection::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr img_ptr;
  try
  {
    img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = img_ptr->image;
    findCenters(image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void RedBallDetection::findCenters(cv::Mat image)
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

  for (int i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(contours[i], contours_poly[i], 5, true);
    bound_rect[i] = cv::boundingRect(contours_poly[i]);
    cv::minEnclosingCircle(contours_poly[i], center[i], radius[i]);
  }

  if (center.size() == 0)
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", red_ball).toImageMsg();
    output_pub_.publish(msg);
    return;
  }

  rovi2_msgs::points2d vector_of_points;
  for (int i = 0; i < center.size(); i++)
  {
    // Create msg
    rovi2_msgs::point2d coordinates;

    coordinates.object_id = i;
    coordinates.x = center[i].x;
    coordinates.y = center[i].y;

    vector_of_points.points.push_back(coordinates);
  }
  // Publish points
  vector_of_points.header.stamp = ros::Time::now();
  points_pub_.publish(vector_of_points);

  // Publish image
  if (publish_result_image_)
  {
    // Draw
    cv::Mat box = cv::Mat::zeros(red_ball.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++)
    {
      drawContours(box, contours, -1, cv::Scalar(0, 0, 255), 1);
      rectangle(box, bound_rect[i], cv::Scalar(0, 255, 255), 1, 8, 0);
      circle(box, center[i], radius[i], cv::Scalar(255, 0, 0), 1, 8, 0);
    }

    // Publish
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", box).toImageMsg();
    output_pub_.publish(msg);
  }
}
}  // namespace stereo_vision_avd
