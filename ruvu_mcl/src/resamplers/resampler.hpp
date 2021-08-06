// Copyright 2021 RUVU Robotics B.V.

#pragma once

// forward declare
class ParticleFilter;

class Resampler
{
public:
  virtual ~Resampler();

  /*
   * @brief Resample a particle filter
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  virtual bool resample(ParticleFilter * pf, int needed_particles) = 0;
};
