// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include "../config.hpp"
#include "./adaptive_method.hpp"

// forward declare
struct KLDSamplingConfig;
class ParticleFilter;

class KLDSampling : public AdaptiveMethod
{
public:
  KLDSampling(const KLDSamplingConfig & config);
  int calc_needed_particles(ParticleFilter * pf);

private:
  int calc_n_particles(int n_bins);
  const KLDSamplingConfig config_;
};
