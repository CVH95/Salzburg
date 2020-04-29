/**
 * @file   red_ball_detection.h
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#ifndef RED_BALL_DETECTION
#define RED_BALL_DETECTION

#include <string>
#include <vector>
#include <iostream>

#include <ros/package.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "rovi2_msgs/point2d.h"
#include "rovi2_msgs/points2d.h"

namespace stereo_vision_avd
{
class RedBallDetection
{
private:
  // Atributes
  ros::NodeHandle nh_;

  // Pubs & Subs
  ros::Publisher points_pub_, output_pub_;
  ros::Subscriber image_sub_;

  // Params
  std::string points_topic_, output_topic_, subscribe_topic_;
  bool publish_result_image_;

  // Callback
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  // Detection methods
  void findCenters(cv::Mat frame);

public:
  RedBallDetection(ros::NodeHandle node_handle);
  ~RedBallDetection();

  void initPublishers();
  void initSubscribers();
  bool readParams();
};
}  // namespace stereo_vision_avd

#endif  // RED_BALL_DETECTION