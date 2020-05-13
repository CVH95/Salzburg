/**
 * @file   anytime_planning.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS planner for obstacle avoidance.
 * @brief Uses moveit.
 */

#include "anytime_planning_avd/anytime_planning.h"

namespace anytime_planning_avd
{
AnytimePlanning::AnytimePlanning(ros::NodeHandle node_handle) : nh_(node_handle)
{
  if (!readParams())
  {
    ROS_ERROR("Could not retrieve from parameter server!");
  }
}

AnytimePlanning::~AnytimePlanning()
{
}

bool AnytimePlanning::readParams()
{
  bool success = true;

  return success;
}

void AnytimePlanning::initSubscribers()
{
}

void AnytimePlanning::initPublishers()
{
}
}  // namespace anytime_planning_avd