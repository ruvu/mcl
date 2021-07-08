// Copyright 2021 RUVU Robotics B.V.

#include "./particle_filter.hpp"

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
double ParticleFilter::calc_effective_sample_size()
{
  double sq_weight = 0;
  for (const auto & particle : particles) {
    sq_weight += particle.weight * particle.weight;
  }
  return 1. / sq_weight;
}

std::array<double, 36> ParticleFilter::get_2d_covariance_array()
{
  std::array<double, 36> cov;
  double mean[4] = {0};
  for (const auto & particle : particles) {
    mean[0] += particle.weight * particle.pose.getOrigin().getX();
    mean[1] += particle.weight * particle.pose.getOrigin().getY();
    mean[2] += particle.weight * cos(tf2::getYaw(particle.pose.getRotation()));
    mean[3] += particle.weight * sin(tf2::getYaw(particle.pose.getRotation()));

    // Compute covariance in linear components
    for (size_t j = 0; j < 2; j++) {
      for (size_t k = 0; k < 2; k++) {
        cov[6 * j + k] +=
          particle.weight * particle.pose.getOrigin()[j] * particle.pose.getOrigin()[k];
      }
    }
  }

  // Normalize
  for (size_t j = 0; j < 2; j++) {
    for (size_t k = 0; k < 2; k++) {
      cov[6 * j + k] = cov[6 * j + k] - mean[j] * mean[k];
    }
  }

  // Covariance in angular component
  cov[35] = -2 * log(sqrt(mean[2] * mean[2] + mean[3] * mean[3]));
  return cov;
}
