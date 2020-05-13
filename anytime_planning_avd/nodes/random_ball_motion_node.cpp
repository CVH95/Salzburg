/**
 * @file   red_ball_detection_node.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include <random>
#include <math.h>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "gazebo_msgs/LinkState.h"

double doubleRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void sendTf(geometry_msgs::Pose pose)
{
  geometry_msgs::TransformStamped ts;

  ts.transform.translation.x = pose.position.x;
  ts.transform.translation.y = pose.position.y;
  ts.transform.translation.z = pose.position.z;
  ts.transform.rotation.w = pose.orientation.w;

  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = "world";
  ts.child_frame_id = "red_ball";

  static tf2_ros::StaticTransformBroadcaster br;
  br.sendTransform(ts);
}

int main(int argc, char** argv)
{
  // Seed
  srand(time(NULL));

  ros::init(argc, argv, "red_ball");
  ros::NodeHandle nh;

  std::string topic, link, ref;
  ros::param::get("gazebo_topic", topic);
  ros::param::get("link", link);
  ros::param::get("reference", ref);

  ros::Publisher pub = nh.advertise<gazebo_msgs::LinkState>(topic, 1);

  double x_min = -0.15;
  double y_min = 0.55;
  double z_min = 1.285;

  double x_max = 0.2;
  double y_max = 0.95;
  double z_max = 1.685;

  while (nh.ok())
  {
    gazebo_msgs::LinkState random_state;
    random_state.link_name = link;
    random_state.reference_frame = ref;

    random_state.pose.position.x = doubleRand(x_min, x_max);
    random_state.pose.position.y = doubleRand(y_min, y_max);
    random_state.pose.position.z = doubleRand(z_min, z_max);
    random_state.pose.orientation.x = 0.0;
    random_state.pose.orientation.y = 0.0;
    random_state.pose.orientation.z = 0.0;
    random_state.pose.orientation.w = 1.0;

    random_state.twist.angular.x = 0.0;
    random_state.twist.angular.y = 0.0;
    random_state.twist.angular.z = 0.0;
    random_state.twist.linear.x = 0.0;
    random_state.twist.linear.y = 0.0;
    random_state.twist.linear.z = 0.0;

    pub.publish(random_state);
    sendTf(random_state.pose);
    ROS_INFO("Moving ball to (%f, %f, %f)", random_state.pose.position.x, random_state.pose.position.y,
             random_state.pose.position.z);
    ros::Duration(15.0).sleep();

    ros::spinOnce();
  }

  return 0;
}