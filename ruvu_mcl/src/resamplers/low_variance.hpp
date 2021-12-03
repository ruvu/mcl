// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "./resampler.hpp"

namespace ruvu_mcl
{
// forward declare
class Rng;

/**
 * @brief Implementation of the low-variance sampler
 */
class LowVariance : public Resampler
{
public:
  explicit LowVariance(const std::shared_ptr<Rng> & rng);

  void resample(ParticleFilter * pf, int needed_particles) override;

private:
  std::shared_ptr<Rng> rng_;
};
}  // namespace ruvu_mcl
