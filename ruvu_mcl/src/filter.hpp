// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "./particle_filter.hpp"
#include "ros/message_forward.h"
#include "ros/node_handle.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

// forward declare
class ParticleFilter;
class MotionModel;
struct Map;
class Laser;
class Resampler;
class Rng;

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

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan);
  void map_cb(const nav_msgs::OccupancyGridConstPtr & map);
  void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose);

private:
  tf2::Transform get_odom_pose(const ros::Time & time);
  void publish_particle_cloud(const ros::Time & time);

  // data input
  std::shared_ptr<const tf2_ros::Buffer> buffer_;

  // data output
  ros::Publisher cloud_pub_;
  ros::Publisher pose_pub_;
  tf2_ros::TransformBroadcaster transform_br_;

  // internals
  std::shared_ptr<Rng> rng_;
  std::optional<tf2::Transform> last_odom_pose_;
  ParticleFilter filter_;
  std::unique_ptr<MotionModel> model_;
  std::shared_ptr<Map> map_ = nullptr;
  std::map<std::string, std::unique_ptr<Laser>> lasers_;
  std::unique_ptr<Resampler> resampler_;
  int resample_count_;
};
