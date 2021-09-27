// Copyright 2021 RUVU Robotics B.V.

#pragma once

// forward declare
class ParticleFilter;

class AdaptiveMethod
{
public:
  virtual ~AdaptiveMethod() = default;

  virtual void after_odometry_update(ParticleFilter * pf) {}
  virtual void after_sensor_update(ParticleFilter * pf) {}

  /**
   * @brief Calculate how many particles are wanted
   * @return Number of particles
   */
  virtual int calc_needed_particles(ParticleFilter * pf) = 0;
};
