// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <vector>

#include "ros/message_forward.h"
#include "std_msgs/Header.h"
#include "tf2/LinearMath/Transform.h"

// forward declare
namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(LaserScan)
}

namespace ruvu_mcl
{
class ParticleFilter;

class LaserData
{
public:
  using RangeType = double;

  LaserData(const sensor_msgs::LaserScan & scan, const tf2::Transform & pose);

  double get_angle(std::size_t i) const;

  std_msgs::Header header;
  tf2::Transform pose;  // how the laser is mounted relative to base_link

  double angle_min;
  double angle_increment;
  double range_max;
  std::vector<RangeType> ranges;
};

class Laser
{
public:
  virtual ~Laser();

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  virtual void sensor_update(ParticleFilter * pf, const LaserData & data) = 0;
};
}  // namespace ruvu_mcl
