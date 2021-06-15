// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "../config.hpp"
#include "./laser.hpp"
#include "ros/publisher.h"

// forward declare
struct Map;

class BeamModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  BeamModel(const BeamModelConfig & config, const std::shared_ptr<const Map> & map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  double sensor_update(ParticleFilter * pf, const LaserData & data) override;

private:
  const BeamModelConfig parameters_;
  const std::shared_ptr<const Map> map_;
  ros::Publisher debug_pub_;
};
