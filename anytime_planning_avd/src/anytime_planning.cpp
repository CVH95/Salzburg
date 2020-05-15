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
    exit(1);
  }

  ROS_INFO("--- ANYTIME PLANNER ---");
  ROS_INFO(" * Subscribed to: %s", collision_topic_.c_str());
  ROS_INFO(" * Moveit file: %s", moveit_file_.c_str());
  ROS_INFO(" * Ball size: %f", ball_radius_);
  ROS_INFO(" * Tolerance: %f", tolerance_);
  ROS_INFO(" * Camera used: %s", camera_frame_.c_str());
  ROS_INFO("MoveGroup info:");
  ROS_INFO(" * Planning group: %s", planning_group_.c_str());

  // Read moveit configuration file
  setMoveitConfiguration();

  // Start moveit
  mgp_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_));
  planning_scene_interface_.reset(new moveit::planning_interface::PlanningSceneInterface);
  // joint_model_group_.reset(new const robot_state::JointModelGroup);

  // Set robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();

  // joint_model_group_ = mgp_->getCurrentState()->getJointModelGroup(planning_group_);

  // Configure planning
  mgp_->setPoseReferenceFrame(planning_frame_);
  mgp_->setEndEffectorLink(end_effector_frame_);
  mgp_->setPlannerId(planner_id_);

  // Configure shape primitive
  sphere_prim_.type = sphere_prim_.SPHERE;
  sphere_prim_.dimensions.resize(1);
  sphere_prim_.dimensions[0] = ball_radius_ * tolerance_;

  // Allow RVIZ visualization
  moveit_visual_tools::MoveItVisualTools visual_tools(mgp_->getPlanningFrame().c_str());
  visual_tools.loadRemoteControl();
  visual_tools.trigger();

  ros::Duration(2.0).sleep();  // Just to let everything settle
}

AnytimePlanning::~AnytimePlanning()
{
  ROS_WARN("AnytimePlanning node was killed!");
}

bool AnytimePlanning::readParams()
{
  bool success = true;
  success = success && ros::param::get("planning_group", planning_group_);
  success = success && ros::param::get("collision_topic", collision_topic_);
  success = success && ros::param::get("camera_frame", camera_frame_);
  success = success && ros::param::get("moveit_file", moveit_file_);
  success = success && ros::param::get("ball_radius", ball_radius_);
  success = success && ros::param::get("tolerance", tolerance_);
  return success;
}

void AnytimePlanning::setMoveitConfiguration()
{
  YAML::Node map = YAML::LoadFile(moveit_file_.c_str());
  std::vector<double> pick, place, approach;

  YAML::Node entry = map["planning_frame"];
  planning_frame_ = entry[0]["value"].as<std::string>();
  ROS_INFO(" * Planning reference frame: %s", planning_frame_.c_str());

  entry = map["end_effector_frame"];
  end_effector_frame_ = entry[0]["value"].as<std::string>();
  ROS_INFO(" * End Effector frame: %s", end_effector_frame_.c_str());

  entry = map["planner_id"];
  planner_id_ = entry[0]["value"].as<std::string>();
  ROS_INFO(" * Selected planner: %s", planner_id_.c_str());

  entry = map["home_position"];
  home_position_ = entry[0]["value"].as<std::vector<double>>();
  ROS_INFO(" * Home position: [%f, %f, %f, %f, %f, %f]", home_position_[0], home_position_[1], home_position_[2],
           home_position_[3], home_position_[4], home_position_[5]);

  entry = map["pick"];
  pick = entry[0]["value"].as<std::vector<double>>();
  ROS_INFO(" * Pick position: [%f, %f, %f, %f, %f, %f]", pick[0], pick[1], pick[2], pick[3], pick[4], pick[5]);

  entry = map["place"];
  place = entry[0]["value"].as<std::vector<double>>();
  ROS_INFO(" * Place position: [%f, %f, %f, %f, %f, %f]", place[0], place[1], place[2], place[3], place[4], place[5]);

  entry = map["place_approach"];
  approach = entry[0]["value"].as<std::vector<double>>();
  ROS_INFO(" * Place approach position: [%f, %f, %f, %f, %f, %f]", approach[0], approach[1], approach[2], approach[3],
           approach[4], approach[5]);

  cycle_vec_.push_back(home_position_);
  cycle_vec_.push_back(pick);
  cycle_vec_.push_back(home_position_);
  cycle_vec_.push_back(approach);
  cycle_vec_.push_back(place);
  cycle_vec_.push_back(approach);

  ROS_INFO("Moveit configuration parsed successfully!");
}

void AnytimePlanning::initSubscribers()
{
  collision_sub_ = nh_.subscribe(collision_topic_, 1, &AnytimePlanning::collisionCallback, this);
}

bool AnytimePlanning::moveRobot(std::vector<double> joint_positions)
{
  mgp_->setStartState(*mgp_->getCurrentState());
  mgp_->setJointValueTarget(joint_positions);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (mgp_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_DEBUG("Planning status: %s", success ? "SUCCESS" : "FAILED");
  if (success)
  {
    success = (mgp_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_DEBUG("Moved camera to new pose %s", success ? "SUCCESS" : "FAILED");
  }
  return success;
}

void AnytimePlanning::runDemo()
{
  int cnt = 0;
  int limit = (int)cycle_vec_.size() - 1;
  while (nh_.ok())
  {
    if (cnt > limit)
    {
      cnt = 0;
    }

    bool success = moveRobot(cycle_vec_[cnt]);
    if (success)
    {
      cnt++;
    }
    else
    {
      ROS_WARN("Could not move robot this time!");
    }
  }
}

void AnytimePlanning::collisionCallback(const rovi2_msgs::points3d& msg)
{
  if (msg.points.size() == 0)
  {
    return;
  }

  std::vector<moveit_msgs::CollisionObject> obstacles;
  for (int i = 0; i < msg.points.size(); i++)
  {
    geometry_msgs::Pose pose;
    pose.position.x = msg.points[i].x;
    pose.position.y = msg.points[i].y;
    pose.position.z = msg.points[i].z;
    pose.orientation.w = 1.0;

    moveit_msgs::CollisionObject red_ball;
    red_ball.header.frame_id = camera_frame_;
    red_ball.id = "obstacle_" + std::to_string(i);
    red_ball.primitives.push_back(sphere_prim_);
    red_ball.primitive_poses.push_back(pose);
    red_ball.operation = red_ball.ADD;

    obstacles.push_back(red_ball);
  }

  planning_scene_interface_->addCollisionObjects(obstacles);
}
}  // namespace anytime_planning_avd