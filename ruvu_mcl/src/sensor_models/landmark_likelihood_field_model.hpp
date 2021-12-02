// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "../config.hpp"
#include "./landmark.hpp"
#include "ros/message_forward.h"
#include "ros/publisher.h"

// forward declare
namespace ruvu_mcl_msgs
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

namespace ruvu_mcl
{
/**
 * @brief Impelements the likelihood field range finder model from Probablistic Robotics for landmark measurments
 */
class LandmarkLikelihoodFieldModel : public LandmarkModel
{
public:
  /*
   * @brief LikelihoodFieldModel constructor
   */
  LandmarkLikelihoodFieldModel(
    const LandmarkLikelihoodFieldModelConfig & config, const LandmarkList & landmarks);

  void sensor_update(ParticleFilter * pf, const LandmarkList & data) override;

private:
  const LandmarkLikelihoodFieldModelConfig config_;
  const LandmarkList landmarks_;
  ros::Publisher debug_pub_;
  ros::Publisher statistics_pub_;
};
}  // namespace ruvu_mcl
