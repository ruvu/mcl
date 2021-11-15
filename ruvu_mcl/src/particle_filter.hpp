// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <array>
#include <vector>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf2/LinearMath/Transform.h"

namespace ruvu_mcl
{
class Particle
{
public:
  tf2::Transform pose;
  double_t weight;

  Particle(const tf2::Transform & pose, double weight) : pose(pose), weight(weight) {}
};

class ParticleFilter
{
public:
  std::vector<Particle> particles;

  void normalize_weights();
  void normalize_weights(double total_weight);
  double calc_effective_sample_size() const;
  geometry_msgs::PoseWithCovarianceStamped get_pose_with_covariance_stamped(
    const ros::Time & stamp, const std::string & frame_id) const;
};
}  // namespace ruvu_mcl
