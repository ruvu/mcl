// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <boost/shared_ptr.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "./config.hpp"
#include "./particle_filter.hpp"
#include "ros/message_forward.h"
#include "ros/time.h"

// forward declare
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
namespace std_msgs
{
ROS_DECLARE_MESSAGE(Header)
}

namespace ruvu_mcl
{
class AdaptiveMethod;
class LandmarkModel;
class LandmarkList;
class Laser;
class LaserData;
class MotionModel;
class ParticleFilter;
class Resampler;
class Rng;

class Mcl
{
public:
  Mcl();
  Mcl(std::uint_fast32_t seed);
  ~Mcl();  // to handle forward declares

  void configure(const Config & config);
  const Config & config() const { return config_; }
  const PoseWithCovariance get_pose_with_covariance() const
  {
    return filter_.get_pose_with_covariance();
  }
  const std::vector<Particle> & particles() const { return filter_.particles; }

  bool scan_cb(const LaserData & scan, const tf2::Transform & odom_pose);
  bool landmark_cb(const LandmarkList & landmarks, const tf2::Transform & odom_pose);
  void map_cb(const nav_msgs::OccupancyGridConstPtr & map);
  void landmark_list_cb(const LandmarkList & landmarks);
  void initial_pose_cb(const ros::Time & stamp, const PoseWithCovariance & initial_pose);
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

  Mcl(const std::shared_ptr<Rng> & rng);

  bool odometry_update(
    const std_msgs::Header & header, const MeasurementType & measurement_type,
    tf2::Transform odom_pose);
  bool should_process(const tf2::Transform & diff, const MeasurementKey & measurment_key);

  // internals
  Config config_;
  std::shared_ptr<Rng> rng_;
  std::optional<tf2::Transform> last_odom_pose_;
  ros::Time last_filter_update_;
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
}  // namespace ruvu_mcl
