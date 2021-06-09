#pragma once

#include "./particle_filter.hpp"
#include "message_filters/subscriber.h"
#include "ros/message_forward.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

// forward declare
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

class Node
{
public:
  Node(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~Node();  // to handle forward declares

private:
  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan);
  void map_cb(const nav_msgs::OccupancyGridConstPtr & map);
  void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose);
  tf2::Transform get_odom_pose(const ros::Time & time);
  void publish_particle_cloud(const ros::Time & time);

  // data input
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{tf_buffer};
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_scan_filter_;
  ros::Subscriber map_sub_;
  ros::Subscriber initial_pose_sub_;

  // data output
  ros::Publisher cloud_pub_;

  // internals
  std::shared_ptr<Rng> rng_;
  tf2::Transform last_odom_pose_ = {};
  ParticleFilter particles_ = {};
  std::unique_ptr<MotionModel> model_;
  std::shared_ptr<Map> map_ = nullptr;
  std::map<std::string, std::unique_ptr<Laser>> lasers_;
  std::unique_ptr<Resampler> resampler_;
};
