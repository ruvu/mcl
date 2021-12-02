// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <array>
#include <vector>

#include "tf2/LinearMath/Transform.h"

namespace ruvu_mcl
{
/**
 * @brief This represents a pose in free space with uncertainty.
 */
struct PoseWithCovariance
{
  PoseWithCovariance(const tf2::Transform & pose, const std::array<double, 36> & covariance)
  : pose(pose), covariance(covariance)
  {
  }
  tf2::Transform pose;
  std::array<double, 36> covariance;
};

/**
 * @brief A single position estimate
 */
class Particle
{
public:
  /**
   * @brief The position estimate
   */
  tf2::Transform pose;

  /**
   * @brief How well this position estimate has matched previous measurments
   */
  double_t weight;

  /**
   * @brief Create a particle
   */
  Particle(const tf2::Transform & pose, double weight) : pose(pose), weight(weight) {}
};

/**
 * @brief Hold a vector of Particles
 */
class ParticleFilter
{
public:
  std::vector<Particle> particles;

  /**
   * @brief Normalize the weight of all particles such that the sum is 1
   */
  void normalize_weights();

  /**
   * @brief Normalize the weight of all particles such that the sum is 1
   *
   * This function avoids calculating the total_weight if it is already known.
   */
  void normalize_weights(double total_weight);

  /**
   * @brief Calculate the effective sample size
   *
   * The effective sample size can be used as a criterion for deciding when to perform the
   * resampling step. It is a measure how well the current particle set represents the target
   * posterior.
   */
  double calc_effective_sample_size() const;

  /**
   * @brief Calculate a mean pose with covariance of all the particles in the filter
   */
  PoseWithCovariance get_pose_with_covariance() const;
};
}  // namespace ruvu_mcl
