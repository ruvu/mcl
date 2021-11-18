// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <array>
#include <vector>

#include "tf2/LinearMath/Transform.h"

namespace ruvu_mcl
{
struct PoseWithCovariance
{
  PoseWithCovariance(const tf2::Transform & pose, const std::array<double, 36> & covariance)
  : pose(pose), covariance(covariance)
  {
  }
  tf2::Transform pose;
  std::array<double, 36> covariance;
};

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
  PoseWithCovariance get_pose_with_covariance() const;
};
}  // namespace ruvu_mcl
