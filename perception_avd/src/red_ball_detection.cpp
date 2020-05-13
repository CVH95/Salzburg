/**
 * @file   red_ball_detection.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "perception_avd/red_ball_detection.h"

namespace perception_avd
{
RedBallDetection::RedBallDetection(ros::NodeHandle node_handle) : nh_(node_handle)
{
  if (!readParams())
  {
    ROS_ERROR("Could not load params from paramater server!");
    exit(1);
  }

  ROS_INFO("--- %s ---", stereo_vision_ ? "STEREO DETECTION" : "MONOCULAR DETECTION");
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
  success = success && ros::param::get("stereo_vision", stereo_vision_);
  return success;
}

void RedBallDetection::initSubscribers()
{
  image_sub_ = nh_.subscribe(subscribe_topic_, 1, &RedBallDetection::imageCallback, this);
}

void RedBallDetection::initPublishers()
{
  if (stereo_vision_)
  {
    points_pub_ = nh_.advertise<rovi2_msgs::points2d>(points_topic_, 1);
  }
  else
  {
    points_pub_ = nh_.advertise<rovi2_msgs::boundingBoxes>(points_topic_, 1);
  }

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

    rovi2_msgs::points2d vector_of_points;
    rovi2_msgs::boundingBoxes bb_corners;

    findCenters(image, vector_of_points, bb_corners);

    // Publishing
    if (stereo_vision_)
    {
      points_pub_.publish(vector_of_points);
    }
    else
    {
      points_pub_.publish(bb_corners);
    }

    // Publish image
    if (publish_result_image_)
    {
      // Publish
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      output_pub_.publish(msg);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}
}  // namespace perception_avd
