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

class LandmarkLikelihoodFieldModel
{
public:
  /*
   * @brief LikelihoodFieldModel constructor
   */
  LandmarkLikelihoodFieldModel(
    const LandmarkLikelihoodFieldModelConfig & config, const LandmarkList & landmarks);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  double sensor_update(ParticleFilter * pf, const LandmarkList & data);

private:
  const LandmarkLikelihoodFieldModelConfig config_;
  const LandmarkList landmarks_;
  ros::Publisher debug_pub_;
};
