// Copyright 2021 RUVU Robotics B.V.

#include "./kld_sampling.hpp"

#include <unordered_set>

#include "../particle_filter.hpp"
#include "./utils.hpp"
#include "ros/console.h"
#include "tf2/utils.h"

namespace ruvu_mcl
{
KLDSampling::KLDSampling(const KLDSamplingConfig & config) : config_(config) {}

int KLDSampling::calc_needed_particles(const ParticleFilter & pf) const
{
  // Discretize particles:
  std::unordered_set<Key, KeyHasher> grid_clusters;
  for (auto & particle : pf.particles) {
    int i = particle.pose.getOrigin().getX() / config_.xy_grid_size;
    int j = particle.pose.getOrigin().getY() / config_.xy_grid_size;
    int k = tf2::getYaw(particle.pose.getRotation()) / config_.theta_grid_size;
    grid_clusters.emplace(i, j, k);
  }

  return calc_n_particles(grid_clusters.size());
}

int KLDSampling::calc_n_particles(int n_bins) const
{
  // From the paper: KLD-Sampling: Adaptive Particle Filters
  if (n_bins <= 1) return config_.max_particles;

  double k = n_bins;
  double a = 2 / (9 * (k - 1));
  double b = sqrt(a) * config_.kld_z;
  double c = 1 - a + b;
  int n = ceil((k - 1) / (2 * config_.kld_err) * c * c * c);

  if (n < config_.min_particles) return config_.min_particles;
  if (n > config_.max_particles) return config_.max_particles;
  return n;
}
}  // namespace ruvu_mcl
