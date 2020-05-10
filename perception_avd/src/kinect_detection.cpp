/**
 * @file   kinect_detection.cpp
 *
 * Richárd, Sergi, Mathesh and Carlos.
 *
 * @brief ROS wrapper for detection of (spheric) red obstacles.
 * @brief Publishes image points corresponding to object centers.
 */

#include "perception_avd/kinect_detection.h"

namespace perception_avd
{
KinectDetection::KinectDetection(ros::NodeHandle node_handle) : nh_(node_handle)
{
  if (!readParams())
  {
    ROS_ERROR("Error reading from paramter server!");
    exit(1);
  }

  ROS_INFO("Kinect detection node initialized!");
  ROS_INFO(" * Publishing on topic: %s", pose_topic_.c_str());
  ROS_INFO(" * Input cloud: %s", subscribe_topic_.c_str());
  if (publish_pcl_)
  {
    ROS_INFO(" * Output cloud: %s", result_topic_.c_str());
  }
}

KinectDetection::~KinectDetection()
{
}

bool KinectDetection::readParams()
{
  bool success = true;
  success = success && ros::param::get("pose_topic", pose_topic_);
  success = success && ros::param::get("input_cloud_topic", subscribe_topic_);
  success = success && ros::param::get("camera_frame", camera_frame_);
  success = success && ros::param::get("publish_pcl", publish_pcl_);
  success = success && ros::param::get("output_cloud_topic", result_topic_);
  return success;
}

void KinectDetection::initPublishers()
{
  pose_pub_ = nh_.advertise<rovi2_msgs::points3d>(pose_topic_, 1);

  if (publish_pcl_)
  {
    result_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(result_topic_, 1);
  }
}

void KinectDetection::initSubscribers()
{
  sub_ = nh_.subscribe(subscribe_topic_, 1, &KinectDetection::cloudCallback, this);
}

void KinectDetection::broadcastDetectedTf(rovi2_msgs::point3d p, std::string id)
{
  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = camera_frame_;
  ts.child_frame_id = "detection_" + id;

  ts.transform.translation.x = p.x;
  ts.transform.translation.y = p.y;
  ts.transform.translation.z = p.z;
  ts.transform.rotation.x = 0.0;
  ts.transform.rotation.y = 0.0;
  ts.transform.rotation.z = 0.0;
  ts.transform.rotation.w = 1.0;

  br.sendTransform(ts);
}

pcl::PointCloud<pcl::PointXYZRGB> KinectDetection::colorSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
  // kd-tree object for searches.
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdtree->setInputCloud(cloud);

  // Color-based region growing clustering object.
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
  clustering.setInputCloud(cloud);
  clustering.setSearchMethod(kdtree);
  // Here, the minimum cluster size affects also the postprocessing step:
  // clusters smaller than this will be merged with their neighbors.
  clustering.setMinClusterSize(100);
  // Set the distance threshold, to know which points will be considered neighbors.
  clustering.setDistanceThreshold(10);
  // Color threshold for comparing the RGB color of two points.
  clustering.setPointColorThreshold(6);
  // Region color threshold for the postprocessing step: clusters with colors
  // within the threshold will be merged in one.
  clustering.setRegionColorThreshold(5);

  std::vector<pcl::PointIndices> clusters;
  clustering.extract(clusters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
      cluster->points.push_back(cloud->points[*point]);

    int count_red_points = 0;
    for (int i = 0; i < 20; i++)
    {
      pcl::PointXYZRGB point = cluster->points[i];
      if (point.r > 179 && point.r <= 255)
      {
        if (point.b < 65 && point.g < 60)
        {
          count_red_points++;
        }
      }
    }
    if (count_red_points > 16)
    {
      ROS_INFO("Found red ball");
      return *output_cluster_cloud;
    }
  }

  return *output_cluster_cloud;
}

/*void KinectDetection::objectClustering(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
}*/

Eigen::Vector3d KinectDetection::estimateClusterPose(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(cloud, centroid);

  Eigen::Vector3d final_pose(0, 0, 0);

  final_pose.x() = centroid[0] / centroid[3];
  final_pose.y() = centroid[2] / centroid[3];
  final_pose.z() = centroid[1] / centroid[3];

  return final_pose;
}

void KinectDetection::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (msg->data.empty())
  {
    ROS_ERROR("No cloud data!");
    return;
  }
  // std::cout << "received" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
  pcl::fromROSMsg(*msg, cloud_in);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud = cloud_in.makeShared();

  // std::cout << "converted" << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> red_ball_cluster = colorSegmentation(cloud);
  Eigen::Vector3d ball_xyz = estimateClusterPose(red_ball_cluster);

  rovi2_msgs::points3d obstacle_track;
  rovi2_msgs::point3d p;
  ROS_INFO("Detected obstacle at (%f, %f, %f)", ball_xyz.x(), ball_xyz.y(), ball_xyz.z());

  // Kalman filter
  cv::Mat_<float> measurement(3, 1);
  measurement(0) = ball_xyz.x();
  measurement(1) = ball_xyz.y();
  measurement(2) = ball_xyz.z();

  Eigen::Vector3f track = kalman_->kalmanFilter3d(measurement, kf_);
  rovi2_msgs::point3d k;
  k.x = track.x();
  k.y = track.y();
  k.z = track.z();
  k.object_id = 0;

  broadcastDetectedTf(k, "0");
  obstacle_track.points.push_back(k);
  ROS_INFO("Tracked obstacle at (%f, %f, %f)", k.x, k.y, k.z);

  obstacle_track.header.stamp = ros::Time::now();
  obstacle_track.header.frame_id = camera_frame_;
  pose_pub_.publish(obstacle_track);

  if (publish_pcl_)
  {
    sensor_msgs::PointCloud2 pc_ros;
    pcl::toROSMsg(red_ball_cluster, pc_ros);
    result_pub_.publish(pc_ros);
  }

  ROS_INFO("---------------");
}

}  // namespace perception_avd