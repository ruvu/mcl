// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "../particle_filter.hpp"

class Resampler
{
public:
  virtual ~Resampler();

  /*
   * @brief Resample a particle filter
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  virtual bool resample(ParticleFilter * pf) = 0;
};
