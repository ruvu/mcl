// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "./landmark.hpp"
#include "ros/message_forward.h"
#include "ros/publisher.h"

// forward declare
class GaussianLandmarkModelConfig;
namespace ruvu_mcl_msgs
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

class GaussianLandmarkModel : public LandmarkModel
{
public:
  /*
   * @brief GaussianLandmarkModel constructor
   */
  GaussianLandmarkModel(const GaussianLandmarkModelConfig & config, const LandmarkList & map);

  /*
   * @brief Run a sensor update on the particles with landmark data
   * @param pf Particle filter to use
   * @param data Landmark data to use
   * @return if it was succesful
   */
  void sensor_update(ParticleFilter * pf, const LandmarkList & data) override;

private:
  // parameters
  double z_rand_;
  double landmark_var_r_;
  double landmark_var_t_;
  std::string global_frame_id_;

  const LandmarkList map_;
  ros::Publisher debug_pub_;
  ros::Publisher statistics_pub_;
};
