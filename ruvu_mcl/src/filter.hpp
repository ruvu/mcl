// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "./config.hpp"
#include "./particle_filter.hpp"
#include "ros/message_forward.h"
#include "ros/publisher.h"
#include "tf2_ros/transform_broadcaster.h"

// forward declare
class Laser;
class MotionModel;
class ParticleFilter;
class Resampler;
class Rng;
namespace ros
{
class NodeHandle;
class Time;
}  // namespace ros
namespace tf2
{
class Transform;
}
namespace tf2_ros
{
class Buffer;
}
namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(PoseWithCovarianceStamped)
}
namespace nav_msgs
{
ROS_DECLARE_MESSAGE(OccupancyGrid)
}
namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(LaserScan)
}

class Filter
{
public:
  Filter(
    ros::NodeHandle nh, ros::NodeHandle private_nh,
    const std::shared_ptr<const tf2_ros::Buffer> & buffer);
  ~Filter();  // to handle forward declares

  void configure(const Config & config);

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan);
  void map_cb(const nav_msgs::OccupancyGridConstPtr & map);
  void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose);

private:
  tf2::Transform get_odom_pose(const ros::Time & time);
  void publish_particle_cloud(const ros::Time & time);
  tf2::Transform get_output_pose(const ParticleFilter pf);
  void publish_pose_with_covariance(const tf2::Transform pose);
  void broadcast_tf(
    const tf2::Transform pose, const tf2::Transform odom_pose, const ros::Time stamp);

  // data input
  std::shared_ptr<const tf2_ros::Buffer> buffer_;

  // data output
  ros::Publisher cloud_pub_;
  ros::Publisher pose_pub_;
  tf2_ros::TransformBroadcaster transform_br_;

  // internals
  Config config_;
  std::shared_ptr<Rng> rng_;
  std::optional<tf2::Transform> last_odom_pose_;
  std::optional<tf2::Transform> last_pose_;
  ParticleFilter filter_;
  std::unique_ptr<MotionModel> model_;
  nav_msgs::OccupancyGridConstPtr map_;
  std::map<std::string, std::unique_ptr<Laser>> lasers_;
  std::unique_ptr<Resampler> resampler_;
  int resample_count_;
};
