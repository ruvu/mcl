// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "./cloud_publisher.hpp"
#include "./mcl.hpp"
#include "ros/message_forward.h"
#include "tf2_ros/transform_broadcaster.h"

namespace ros
{
class NodeHandle;
}
namespace tf2_ros
{
class Buffer;
}
namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(PoseWithCovariance)
}
namespace nav_msgs
{
ROS_DECLARE_MESSAGE(OccupancyGrid)
}
namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(LaserScan)
}
namespace ruvu_mcl_msgs
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

namespace ruvu_mcl
{
class AMCLConfig;

/**
 * @brief ROS wrapper for the Mcl class
 *
 * This class is a ROS wrapper for the Mcl class. It receives ROS messages, converts then and sends
 * then trough. It also publishes the output of Mcl to /tf and ~pose.
 */
class MclRos
{
public:
  MclRos(
    ros::NodeHandle nh, ros::NodeHandle private_nh,
    const std::shared_ptr<const tf2_ros::Buffer> & buffer);
  MclRos(
    ros::NodeHandle nh, ros::NodeHandle private_nh,
    const std::shared_ptr<const tf2_ros::Buffer> & buffer, uint_fast32_t seed);
  ~MclRos();  // to handle forward declares

  void configure(const ruvu_mcl::AMCLConfig & config);
  const geometry_msgs::PoseWithCovariance get_pose_with_covariance() const;

  bool scan_cb(const sensor_msgs::LaserScanConstPtr & scan);
  bool landmark_cb(const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks);
  void map_cb(const nav_msgs::OccupancyGridConstPtr & map);
  void landmark_list_cb(const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks);
  void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose);

private:
  tf2::Transform get_odom_pose(const ros::Time & time) const;
  void broadcast_tf(
    const tf2::Transform & pose, const tf2::Transform & odom_pose, const ros::Time & stamp);
  void broadcast_last_tf(const ros::Time & stamp);
  void publish_data(const ros::Time & stamp, const PoseWithCovariance & pose_with_covariance);

  Mcl mcl_;
  std::shared_ptr<const tf2_ros::Buffer> buffer_;
  tf2_ros::TransformBroadcaster transform_br_;
  geometry_msgs::TransformStamped last_tf_broadcast_;
  CloudPublisher cloud_pub_;
  ros::Publisher count_pub_;
  ros::Publisher pose_pub_;
};
}  // namespace ruvu_mcl
