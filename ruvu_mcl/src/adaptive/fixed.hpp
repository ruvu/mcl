// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include <cstddef>

#include "./adaptive_method.hpp"

// forward declare
class ParticleFilter;
class Config;

class Fixed : public AdaptiveMethod
{
public:
  Fixed(const Config & config);

  void before_odometry_update(ParticleFilter * pf) override;

private:
  size_t max_particles_;
};
