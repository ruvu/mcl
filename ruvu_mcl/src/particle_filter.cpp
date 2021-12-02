// Copyright 2021 RUVU Robotics B.V.

#include "./particle_filter.hpp"

#include "ros/console.h"

constexpr auto name = "particle_filter";

namespace ruvu_mcl
{
void ParticleFilter::normalize_weights()
{
  double total_weight = 0;
  for (const auto & particle : particles) {
    total_weight += particle.weight;
  }
  normalize_weights(total_weight);
}

void ParticleFilter::normalize_weights(double total_weight)
{
  if (total_weight > 0) {
    for (auto & particle : particles) {
      particle.weight /= total_weight;
    }
  } else {
    // total_weight could be 0 due to very strange data input. We should print an error and
    // continue. If total_weight is negative or NaN, there is a bigger problem.
    assert(total_weight == 0);
    ROS_ERROR_NAMED(
      name,
      "Total weight of particles is 0. This probably means there is a bug in one of the sensor "
      "models or they are configured incorrectly. Resetting particles to uniform weight "
      "distribution");
    for (auto & particle : particles) {
      particle.weight = 1.0 / particles.size();
    }
  }
}

// https://people.eecs.berkeley.edu/~pabbeel/cs287-fa13/optreadings/GrisettiStachnissBurgard_gMapping_T-RO2006.pdf
double ParticleFilter::calc_effective_sample_size() const
{
  double sq_weight = 0;
  for (const auto & particle : particles) {
    sq_weight += particle.weight * particle.weight;
  }
  return 1. / sq_weight;
}

PoseWithCovariance ParticleFilter::get_pose_with_covariance() const
{
  std::array<double, 36> cov = {0};
  double mean[4] = {0};

  for (const auto & particle : particles) {
    // the four components of mean are: average x position, average y position,
    // average direction x-component and average direction y-component
    mean[0] += particle.weight * particle.pose.getOrigin().getX();
    mean[1] += particle.weight * particle.pose.getOrigin().getY();
    mean[2] += particle.weight * particle.pose.getBasis().getColumn(0).getX();
    mean[3] += particle.weight * particle.pose.getBasis().getColumn(0).getY();

    // Compute covariance in linear components
    for (size_t j = 0; j < 2; j++) {
      for (size_t k = 0; k < 2; k++) {
        cov[6 * j + k] +=
          particle.weight * particle.pose.getOrigin()[j] * particle.pose.getOrigin()[k];
      }
    }
  }

  // Normalize linear covariance
  for (size_t j = 0; j < 2; j++) {
    for (size_t k = 0; k < 2; k++) {
      cov[6 * j + k] = cov[6 * j + k] - mean[j] * mean[k];
    }
  }

  // Covariance in angular component
  cov[35] = -2 * log(sqrt(mean[2] * mean[2] + mean[3] * mean[3]));

  // Mean pose
  tf2::Vector3 agv_p{mean[0], mean[1], 0};
  tf2::Quaternion avg_q;
  avg_q.setRPY(0, 0, atan2(mean[3], mean[2]));
  return PoseWithCovariance{tf2::Transform{avg_q, agv_p}, cov};
}
}  // namespace ruvu_mcl
