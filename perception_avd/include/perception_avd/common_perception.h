/**
 * @file   common_perception.h
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include <string>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/opencv.hpp>

#include "rovi2_msgs/point2d.h"
#include "rovi2_msgs/point3d.h"
#include "rovi2_msgs/points2d.h"
#include "rovi2_msgs/points3d.h"
#include "rovi2_msgs/boundingBox.h"
#include "rovi2_msgs/boundingBoxes.h"
#include "geometry_msgs/TransformStamped.h"

namespace perception_avd
{
// Red ball detection method
void findCenters(cv::Mat& image, rovi2_msgs::points2d& vector_of_points, rovi2_msgs::boundingBoxes& bb_corners);

// Bounding box definition
std::vector<cv::Point2f> getCornersFromRect(cv::Rect rectangle);
rovi2_msgs::boundingBox getBoxFromRect(cv::Rect rectangle);

// Monocular triangulation
std::vector<cv::Point2f> getCornersFromBox(rovi2_msgs::boundingBox bb);

// Utils
void printMatrix(cv::Mat matrix);
void broadcastDetectedTf(rovi2_msgs::point3d p, std::string id, std::string camera_frame);
float euclideanDistance(rovi2_msgs::point2d left_point, rovi2_msgs::point2d right_point);
}  // namespace perception_avd