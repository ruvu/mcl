// Copyright 2021 RUVU Robotics B.V.

#include "./particle_filter.hpp"

// https://people.eecs.berkeley.edu/~pabbeel/cs287-fa13/optreadings/GrisettiStachnissBurgard_gMapping_T-RO2006.pdf
double ParticleFilter::calc_effective_sample_size()
{
  double sq_weight = 0;
  for (const auto & particle : particles) {
    sq_weight += particle.weight * particle.weight;
  }
  return 1. / sq_weight;
}
