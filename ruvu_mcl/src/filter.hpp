// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>
#include <string>

#include "./cloud_publisher.hpp"
#include "./config.hpp"
#include "./particle_filter.hpp"
#include "ros/message_forward.h"
#include "ros/publisher.h"
#include "tf2_ros/transform_broadcaster.h"

// forward declare
class AdaptiveMethod;
class LandmarkLikelihoodFieldModel;
class LandmarkList;
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
namespace ruvu_mcl_msgs
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

std::unique_ptr<Laser> create_laser_model(Config config, nav_msgs::OccupancyGridConstPtr map);

class Filter
{
public:
  Filter(
    ros::NodeHandle nh, ros::NodeHandle private_nh,
    const std::shared_ptr<const tf2_ros::Buffer> & buffer);
  ~Filter();  // to handle forward declares

  void configure(const Config & config);

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan);
  void landmark_cb(const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks);
  void map_cb(const nav_msgs::OccupancyGridConstPtr & map);
  void landmark_list_cb(const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks);
  void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose);

private:
  enum class MeasurementType { LASER, LANDMARK };

  /**
   * For each x meters moved, each sensor should be processed once. To do this a map will be
   * constructed on MeasurementKey. This means measurments from the same frame_id, but with a
   * different measurement type will still be processed.
   */
  using MeasurementKey = std::tuple<MeasurementType, std::string>;

  friend std::ostream & operator<<(std::ostream & out, const MeasurementType & measurement_type);

  bool odometry_update(const std_msgs::Header & header, const MeasurementType measurement_type);
  tf2::Transform get_odom_pose(const ros::Time & time);
  bool should_process(const tf2::Transform & diff, const MeasurementKey & measurment_key);
  void publish_data(const geometry_msgs::PoseWithCovarianceStamped & ps);
  void broadcast_tf(
    const tf2::Transform pose, const tf2::Transform odom_pose, const ros::Time stamp);

  // data input
  std::shared_ptr<const tf2_ros::Buffer> buffer_;

  // data output
  CloudPublisher cloud_pub_;
  ros::Publisher count_pub_;
  ros::Publisher pose_pub_;
  tf2_ros::TransformBroadcaster transform_br_;

  // internals
  Config config_;
  std::shared_ptr<Rng> rng_;
  std::optional<tf2::Transform> last_odom_pose_;
  tf2::Transform last_pose_;
  ParticleFilter filter_;
  std::unique_ptr<MotionModel> model_;
  nav_msgs::OccupancyGridConstPtr map_;
  std::shared_ptr<LandmarkList> landmarks_;
  std::unique_ptr<Laser> laser_;
  std::unique_ptr<LandmarkLikelihoodFieldModel> landmark_model_;

  /**
   * @brief If a measurement from a frame_id should be processed
   */
  std::map<MeasurementKey, bool> should_process_;
  std::unique_ptr<Resampler> resampler_;
  int resample_count_;
  std::unique_ptr<AdaptiveMethod> adaptive_;
};

std::ostream & operator<<(std::ostream & out, const Filter::MeasurementType & measurement_type);
