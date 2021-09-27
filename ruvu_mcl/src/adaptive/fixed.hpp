// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include <cstddef>

#include "./adaptive_method.hpp"

// forward declare
class ParticleFilter;
struct Config;

class Fixed : public AdaptiveMethod
{
public:
  Fixed(const Config & config);
  int calc_needed_particles(ParticleFilter * pf);

private:
  size_t max_particles_;
};
