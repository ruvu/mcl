// Copyright 2021 RUVU Robotics B.V.

#include "./low_variance.hpp"

#include <memory>

#include "../particle_filter.hpp"
#include "../rng.hpp"
#include "ros/console.h"

constexpr auto name = "low_variance";

LowVariance::LowVariance(std::shared_ptr<Rng> rng) : rng_(rng) {}

bool LowVariance::resample(ParticleFilter * pf, int needed_particles)
{
  assert(pf->particles.size());
  ROS_DEBUG_NAMED(name, "resample");
  // Low-variance resampling (Page 86 Probabilistc Robotics)
  double step_size = 1. / needed_particles;
  auto uniform_dist = rng_->uniform_distribution(0, step_size);

  ParticleFilter pf_resampled;
  pf_resampled.particles.reserve(needed_particles);
  float r = uniform_dist();
  double c = pf->particles.at(0).weight;
  int i = 0;
  for (int m = 0; m < needed_particles; m++) {
    float u = r + m * step_size;
    while (u > c) {
      i++;
      c += pf->particles.at(i).weight;
    }
    pf_resampled.particles.emplace_back(pf->particles.at(i));
  }
  pf_resampled.normalize_weights();
  *pf = pf_resampled;
  return true;
}
