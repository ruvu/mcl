#pragma once

#include <vector>

#include "../particle_filter.hpp"

class Range
{
};

class LaserData
{
public:
  std::vector<Range> ranges;
};

class Laser
{
public:
  virtual ~Laser();

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  virtual bool sensor_update(ParticleFilter * pf, const LaserData & data) = 0;
};
