// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "../config.hpp"
#include "./laser.hpp"
#include "ros/publisher.h"

// forward declare
struct DistanceMap;

class LikelihoodFieldModel : public Laser
{
public:
  /*
   * @brief LikelihoodFieldModel constructor
   */
  LikelihoodFieldModel(
    const LikelihoodFieldModelConfig & config, const std::shared_ptr<const DistanceMap> & map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  void sensor_update(ParticleFilter * pf, const LaserData & data) override;

private:
  const LikelihoodFieldModelConfig config_;
  const std::shared_ptr<const DistanceMap> map_;
  ros::Publisher debug_pub_;
  ros::Publisher statistics_pub_;
};
