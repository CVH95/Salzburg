/**
 * @file   anytime_planning.h
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS planner for obstacle avoidance.
 * @brief Uses moveit.
 */

#ifndef ANYTIME_PLANNING
#define ANYTIME_PLANNING

#include <string>
#include <vector>
#include <iostream>

#include <ros/package.h>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include "rovi2_msgs/points3d.h"
#include "geometry_msgs/TransformStamped.h"

namespace anytime_planning_avd
{
class AnytimePlanning
{
private:
  ros::NodeHandle nh_;

public:
  AnytimePlanning(ros::NodeHandle node_handle);
  ~AnytimePlanning();

  bool readParams();
  void initSubscribers();
  void initPublishers();
};
}  // namespace anytime_planning_avd

#endif  // ANYTIME_PLANNING