// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <memory>

#include "../config.hpp"
#include "./laser.hpp"
#include "ros/publisher.h"

namespace ruvu_mcl
{
// forward declare
struct DistanceMap;

/**
 * @brief Impelements the likelihood field range finder model from Probablistic Robotics
 */
class LikelihoodFieldModel : public Laser
{
public:
  /*
   * @brief LikelihoodFieldModel constructor
   */
  LikelihoodFieldModel(
    const LikelihoodFieldModelConfig & config, const std::shared_ptr<const DistanceMap> & map);

  void sensor_update(ParticleFilter * pf, const LaserData & data) override;

private:
  const LikelihoodFieldModelConfig config_;
  const std::shared_ptr<const DistanceMap> map_;
  ros::Publisher debug_pub_;
  ros::Publisher statistics_pub_;
};
}  // namespace ruvu_mcl
