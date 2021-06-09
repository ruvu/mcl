#include "./low_variance.hpp"

#include "../rng.hpp"
#include "ros/console.h"

auto normalize_weights(ParticleFilter * pf)
{
  double total_weight = 0;
  for (const auto & particle : *pf) {
    total_weight += particle.weight;
  }
  for (auto & particle : *pf) {
    particle.weight /= total_weight;
  }
}

LowVariance::LowVariance(std::shared_ptr<Rng> rng) : rng_(rng) {}

bool LowVariance::resample(ParticleFilter * pf)
{
  ROS_DEBUG("LowVariance::resample");
  // Low-variance resampling (Page 86 Probabilistc Robotics)
  int M = pf->size();
  double M_inv = 1. / M;
  auto uniform_dist = [this, M_inv]() { return rng_->sample_uniform_distribution(0, M_inv); };

  ParticleFilter pf_resampled;
  float r = uniform_dist();
  double c = pf->at(0).weight;
  int i = 0;
  for (int m = 0; m < M; m++) {
    float u = r + m * M_inv;
    while (u > c) {
      i++;
      c += pf->at(i).weight;
    }
    pf_resampled.emplace_back(pf->at(i));
  }
  normalize_weights(&pf_resampled);
  *pf = pf_resampled;
  return true;
}