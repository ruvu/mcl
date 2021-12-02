// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include <cstddef>

#include "./adaptive_method.hpp"

namespace ruvu_mcl
{
// forward declare
class ParticleFilter;
struct Config;

/**
 * @brief Use a fixed amount of particles (not adaptive)
 */
class Fixed : public AdaptiveMethod
{
public:
  explicit Fixed(const Config & config);
  int calc_needed_particles(const ParticleFilter & pf) const override;

private:
  size_t max_particles_;
};
}  // namespace ruvu_mcl
