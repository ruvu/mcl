// Copyright 2021 RUVU Robotics B.V.

#pragma once

namespace ruvu_mcl
{
// forward declare
class ParticleFilter;

/**
 * @brief Adaptive methods changes the efficiency of filtering by adapting the number of particles.
 */
class AdaptiveMethod
{
public:
  virtual ~AdaptiveMethod() = default;

  /**
   * @brief Hook that runs before the odometry update
   */
  virtual void after_odometry_update(ParticleFilter * pf) const {}

  /**
   * @brief Hook that runs after the sensor update
   */
  virtual void after_sensor_update(ParticleFilter * pf) const {}

  /**
   * @brief Calculate how many particles are wanted
   *
   * This number will be used as a target when resampling
   * @return Number of particles
   */
  virtual int calc_needed_particles(const ParticleFilter & pf) const = 0;
};
}  // namespace ruvu_mcl
