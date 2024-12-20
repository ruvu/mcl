// Copyright 2021 RUVU Robotics B.V.

#include "./low_variance.hpp"

#include <memory>

#include "../particle_filter.hpp"
#include "../rng.hpp"
#include "ros/console.h"

constexpr auto name = "low_variance";

namespace ruvu_mcl
{
LowVariance::LowVariance(const std::shared_ptr<Rng> & rng) : rng_(rng) {}

void LowVariance::resample(ParticleFilter * pf, int needed_particles)
{
  assert(!pf->particles.empty());
  ROS_DEBUG_NAMED(name, "resample");
  // Low-variance resampling (Page 86 Probabilistc Robotics)
  double step_size = 1. / needed_particles;
  auto gen_uniform = rng_->uniform_distribution(0, step_size);

  ParticleFilter pf_resampled;
  pf_resampled.particles.reserve(needed_particles);
  double r = gen_uniform();
  double c = pf->particles.at(0).weight;
  int i = 0;
  for (int m = 0; m < needed_particles; m++) {
    double u = r + m * step_size;
    while (u > c) {
      i++;
      c += pf->particles.at(i).weight;
    }
    pf_resampled.particles.emplace_back(pf->particles.at(i));
  }
  pf_resampled.normalize_weights();
  *pf = pf_resampled;
}
}  // namespace ruvu_mcl
