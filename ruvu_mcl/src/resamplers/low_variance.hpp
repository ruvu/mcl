#pragma once

#include <gnuplot-iostream.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include "./resampler.hpp"
#include "ros/console.h"

class LowVariance : public Resampler
{
public:
  /*
   * @brief Resample a particle filter using the low-variance method
   * @param pf Particle filter to use
   * @return if it was succesful
   */
  bool resample(ParticleFilter * pf) override;
};
