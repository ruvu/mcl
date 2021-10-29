// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>
#include <string>

#include "./cloud_publisher.hpp"
#include "./config.hpp"
#include "./particle_filter.hpp"
#include "ros/message_forward.h"
#include "ros/publisher.h"
#include "tf2/transform_datatypes.h"

// forward declare
class AdaptiveMethod;
class LandmarkModel;
class LandmarkList;
class Laser;
class LaserData;
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
namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(PoseWithCovarianceStamped)
}
namespace nav_msgs
{
ROS_DECLARE_MESSAGE(OccupancyGrid)
}

class Mcl
{
public:
  Mcl(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~Mcl();  // to handle forward declares

  void configure(const Config & config);
  const Config & config() const { return config_; }
  const tf2::Transform & pose() const { return last_pose_; }

  bool scan_cb(const LaserData & scan, const tf2::Transform & odom_pose);
  bool landmark_cb(const LandmarkList & landmarks, const tf2::Transform & odom_pose);
  void map_cb(const nav_msgs::OccupancyGridConstPtr & map);
  void landmark_list_cb(const LandmarkList & landmarks);
  void initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose);
  void request_nomotion_update();

private:
  enum class MeasurementType { LASER, LANDMARK };

  /**
   * For each x meters moved, each sensor should be processed once. To do this a map will be
   * constructed on MeasurementKey. This means measurments from the same frame_id, but with a
   * different measurement type will still be processed.
   */
  using MeasurementKey = std::tuple<MeasurementType, std::string>;

  friend std::ostream & operator<<(std::ostream & out, const MeasurementType & measurement_type);

  bool odometry_update(
    const std_msgs::Header & header, const MeasurementType & measurement_type,
    tf2::Transform odom_pose);
  bool should_process(const tf2::Transform & diff, const MeasurementKey & measurment_key);
  void publish_data(const geometry_msgs::PoseWithCovarianceStamped & ps);

  // data output
  CloudPublisher cloud_pub_;
  ros::Publisher count_pub_;
  ros::Publisher pose_pub_;

  // internals
  Config config_;
  std::shared_ptr<Rng> rng_;
  std::optional<tf2::Transform> last_odom_pose_;
  tf2::Stamped<tf2::Transform> last_pose_;
  ParticleFilter filter_;
  std::unique_ptr<MotionModel> model_;
  nav_msgs::OccupancyGridConstPtr map_;
  std::shared_ptr<LandmarkList> landmarks_;
  std::unique_ptr<Laser> laser_;
  std::unique_ptr<LandmarkModel> landmark_model_;

  /**
   * @brief If a measurement from a frame_id should be processed
   */
  std::map<MeasurementKey, bool> should_process_;
  std::unique_ptr<Resampler> resampler_;
  int resample_count_;
  std::unique_ptr<AdaptiveMethod> adaptive_;
};

std::ostream & operator<<(std::ostream & out, const Mcl::MeasurementType & measurement_type);
