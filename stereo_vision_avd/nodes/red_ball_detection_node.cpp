/**
 * @file   red_ball_detection_node.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "stereo_vision_avd/red_ball_detection.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detector");
  ros::NodeHandle nh;

  stereo_vision_avd::RedBallDetection detector(nh);

  detector.initPublishers();
  detector.initSubscribers();

  ros::spin();
  return 0;
}