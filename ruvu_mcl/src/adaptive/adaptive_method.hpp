// Copyright 2021 RUVU Robotics B.V.

#pragma once

// forward declare
class ParticleFilter;

class AdaptiveMethod
{
public:
  virtual void before_odometry_update(ParticleFilter * pf){};
  virtual void after_odometry_update(ParticleFilter * pf){};
  virtual void after_sensor_update(ParticleFilter * pf){};
};
