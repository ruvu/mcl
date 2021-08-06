// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "./resampler.hpp"

// forward declare
class Rng;

class LowVariance : public Resampler
{
public:
  LowVariance(std::shared_ptr<Rng> rng);
  /*
   * @brief Resample a particle filter using the low-variance method
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  bool resample(ParticleFilter * pf, int needed_particles) override;

private:
  std::shared_ptr<Rng> rng_;
};
