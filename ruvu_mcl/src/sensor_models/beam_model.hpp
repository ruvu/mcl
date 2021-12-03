// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "../config.hpp"
#include "./laser.hpp"
#include "ros/publisher.h"

namespace ruvu_mcl
{
// forward declare
struct OccupancyMap;

/**
 * @brief Implements the beam range finder model from Probablistic Robotics
 */
class BeamModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  BeamModel(const BeamModelConfig & config, const std::shared_ptr<const OccupancyMap> & map);

  void sensor_update(ParticleFilter * pf, const LaserData & data) override;

private:
  const BeamModelConfig parameters_;
  const std::shared_ptr<const OccupancyMap> map_;
  ros::Publisher debug_pub_;
  ros::Publisher statistics_pub_;
};
}  // namespace ruvu_mcl
