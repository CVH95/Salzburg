/**
 * @file   anytime_planning_node.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS planner for obstacle avoidance.
 * @brief Uses moveit.
 */

#include "anytime_planning_avd/anytime_planning.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "anytime_planning_node");
  ros::NodeHandle nh;

  anytime_planning_avd::AnytimePlanning planner(nh);

  planner.initPublishers();
  planner.initSubscribers();

  ros::spin();
  return 0;
}