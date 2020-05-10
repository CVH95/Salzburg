/**
 * @file   kinect_detection.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "perception_avd/kinect_detection.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_detection");
  ros::NodeHandle nh;

  perception_avd::KinectDetection detector(nh);

  detector.initPublishers();
  detector.initSubscribers();

  ros::spin();
  return 0;
}