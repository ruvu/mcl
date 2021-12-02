// Copyright 2021 RUVU Robotics B.V.

#pragma once

namespace ruvu_mcl
{
// forward declare
class ParticleFilter;

/**
 * @brief Base class for all resampling algorithms
 *
 * The resampling step has the important function to force particles back to the posterior belief.
 * See also Probablistic robotics for this.
 */
class Resampler
{
public:
  virtual ~Resampler();

  /**
   * @brief Resample the particles of a particle filter
   * @param pf Particle filter to use
   * @param needed_particles How many particles are needed after resampling
   */
  virtual void resample(ParticleFilter * pf, int needed_particles) = 0;
};
}  // namespace ruvu_mcl
