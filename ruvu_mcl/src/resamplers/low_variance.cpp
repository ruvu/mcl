// Copyright 2021 RUVU Robotics B.V.

#include "./low_variance.hpp"

#include <memory>

#include "../particle_filter.hpp"
#include "../rng.hpp"
#include "ros/console.h"

constexpr auto name = "low_variance";

LowVariance::LowVariance(std::shared_ptr<Rng> rng) : rng_(rng) {}

bool LowVariance::resample(ParticleFilter * pf)
{
  assert(pf->particles.size());
  ROS_DEBUG_NAMED(name, "resample");
  // Low-variance resampling (Page 86 Probabilistc Robotics)
  int M = pf->particles.size();
  double M_inv = 1. / M;
  auto uniform_dist = rng_->uniform_distribution(0, M_inv);

  ParticleFilter pf_resampled;
  pf_resampled.particles.reserve(M);
  float r = uniform_dist();
  double c = pf->particles.at(0).weight;
  int i = 0;
  for (int m = 0; m < M; m++) {
    float u = r + m * M_inv;
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
