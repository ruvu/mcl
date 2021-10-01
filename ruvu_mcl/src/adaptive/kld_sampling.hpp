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
  explicit KLDSampling(const KLDSamplingConfig & config);
  int calc_needed_particles(const ParticleFilter & pf) const override;

private:
  int calc_n_particles(int n_bins) const;
  const KLDSamplingConfig config_;
};
