// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <vector>

#include "ros/message_forward.h"
#include "tf2/LinearMath/Transform.h"

// forward declare
class ParticleFilter;

namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(LaserScan)
}
namespace ruvu_mcl_msgs
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

struct Landmark
{
  Landmark(const tf2::Transform & pose) : pose(pose) {}

  /**
   * Landmarks that are from the landmark map are assumed to be transformed to the global frame.
   * Landmark from measurments are assumed to be transformed to the robot frame.
   */
  tf2::Transform pose;
};

class LandmarkList
{
public:
  LandmarkList(const ruvu_mcl_msgs::LandmarkList & msg);

  std::vector<Landmark> landmarks;
};
