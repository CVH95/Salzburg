/**
 * @file   stereopsis_node.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "perception_avd/stereopsis.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "triangulation");
  ros::NodeHandle nh;

  perception_avd::Stereopsis monocular_stereopsis(nh);

  monocular_stereopsis.initPublishers();
  monocular_stereopsis.initSubscribers();

  ros::spin();
  return 0;
}