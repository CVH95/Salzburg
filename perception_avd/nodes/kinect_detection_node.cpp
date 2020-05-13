/**
 * @file   kinect_detection_node.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "perception_avd/kinect_detection.h"

#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_detection");
  ros::NodeHandle nh;

  perception_avd::KinectDetection detector(nh);
  detector.initPublishers();

  message_filters::Subscriber<sensor_msgs::Image> rgb_imgs(nh, detector.image_topic_, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_imgs(nh, detector.depth_topic_, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_imgs, depth_imgs);

  sync.registerCallback(boost::bind(&perception_avd::KinectDetection::synchronizedCallback, detector, _1, _2));

  ROS_INFO("Initialized Register Callback");

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}