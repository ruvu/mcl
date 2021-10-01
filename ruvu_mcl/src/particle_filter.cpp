// Copyright 2021 RUVU Robotics B.V.

#include "./particle_filter.hpp"

#include "algorithm"
#include "string"
#include "tf2/utils.h"

void ParticleFilter::normalize_weights()
{
  double total_weight = 0;
  for (const auto & particle : particles) {
    total_weight += particle.weight;
  }
  for (auto & particle : particles) {
    particle.weight /= total_weight;
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

geometry_msgs::PoseWithCovarianceStamped ParticleFilter::get_pose_with_covariance_stamped(
  const ros::Time & stamp, const std::string & frame_id) const
{
  std::array<double, 36> cov = {0};
  double mean[4] = {0};
  geometry_msgs::PoseWithCovarianceStamped avg_pose;
  avg_pose.header.stamp = stamp;
  avg_pose.header.frame_id = frame_id;

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
  static_assert(cov.size() == avg_pose.pose.covariance.size());
  std::copy(cov.begin(), cov.end(), avg_pose.pose.covariance.begin());

  // Mean pose
  avg_pose.pose.pose.position.x = mean[0];
  avg_pose.pose.pose.position.y = mean[1];
  tf2::Quaternion avg_q;
  avg_q.setRPY(0, 0, atan2(mean[3], mean[2]));
  tf2::convert(avg_q, avg_pose.pose.pose.orientation);
  return avg_pose;
}
