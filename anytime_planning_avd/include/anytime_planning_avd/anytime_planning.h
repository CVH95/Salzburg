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
#include <yaml-cpp/yaml.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "rovi2_msgs/points3d.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"

namespace anytime_planning_avd
{
class AnytimePlanning
{
private:
  ros::NodeHandle nh_;

  // Pubs & Subs
  ros::Subscriber collision_sub_;
  ros::Publisher pub;

  // Params
  std::string planning_group_, collision_topic_, camera_frame_, moveit_file_;
  double ball_radius_, tolerance_;

  // Moveit
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> mgp_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  // std::unique_ptr<const robot_state::JointModelGroup> joint_model_group_;
  robot_model::RobotModelPtr robot_model_;
  std::string planning_frame_, end_effector_frame_, planner_id_;
  std::vector<double> home_position_;
  std::vector<std::vector<double> > cycle_vec_;

  // Red Ball
  shape_msgs::SolidPrimitive sphere_prim_;

  // Callbacks
  void collisionCallback(const rovi2_msgs::points3d& msg);

  // Private methods
  bool moveRobot(std::vector<double> joint_positions);

public:
  AnytimePlanning(ros::NodeHandle node_handle);
  ~AnytimePlanning();

  bool readParams();
  void setMoveitConfiguration();
  void initSubscribers();

  void runDemo();
};
}  // namespace anytime_planning_avd

#endif  // ANYTIME_PLANNING