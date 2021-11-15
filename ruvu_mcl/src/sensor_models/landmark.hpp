// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <vector>

#include "ros/message_forward.h"
#include "std_msgs/Header.h"
#include "tf2/LinearMath/Transform.h"

namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(LaserScan)
}
namespace ruvu_mcl_msgs
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

namespace ruvu_mcl
{
// forward declare
class ParticleFilter;

struct Landmark
{
  explicit Landmark(const tf2::Transform & pose) : pose(pose), id(0) {}
  Landmark(const tf2::Transform & pose, int id) : pose(pose), id(id) {}

  /**
   * Landmarks that are from the landmark map are assumed to be transformed to the global frame.
   * Landmark from measurements are assumed to be in the sensor frame.
   */
  tf2::Transform pose;
  int id;
};

class LandmarkList
{
public:
  LandmarkList() : header(), pose(tf2::Transform::getIdentity()), landmarks() {}

  explicit LandmarkList(
    const ruvu_mcl_msgs::LandmarkList & msg,
    const tf2::Transform & pose = tf2::Transform::getIdentity());

  std_msgs::Header header;
  /**
   * LandmarkLists from measurements should contain a pose with the transform from robot frame to sensor frame.
   */
  tf2::Transform pose;
  std::vector<Landmark> landmarks;
};

class LandmarkModel
{
public:
  virtual ~LandmarkModel();

  /*
   * @brief Run a sensor update on the particles with landmark data
   * @param pf Particle filter to use
   * @param data Landmark data to use
   * @return if it was succesful
   */
  virtual void sensor_update(ParticleFilter * pf, const LandmarkList & data) = 0;
};
}  // namespace ruvu_mcl
