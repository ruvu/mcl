// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include "../config.hpp"
#include "./adaptive_method.hpp"

namespace ruvu_mcl
{
// forward declare
struct KLDSamplingConfig;
class ParticleFilter;

/**
 * @brief Use KLD sampling to adapt the number of particles
 *
 * From the paper: KLD-Sampling: Adaptive Particle Filters
 */
class KLDSampling : public AdaptiveMethod
{
public:
  explicit KLDSampling(const KLDSamplingConfig & config);
  int calc_needed_particles(const ParticleFilter & pf) const override;

private:
  int calc_n_particles(int n_bins) const;
  const KLDSamplingConfig config_;
};
}  // namespace ruvu_mcl
