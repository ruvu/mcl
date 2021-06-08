#pragma once

#include <vector>

#include "../particle_filter.hpp"
#include "ros/message_forward.h"

// forward declare
namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(LaserScan)
}

class LaserData
{
public:
  double angle_min;
  double angle_increment;
  double range_max;
  std::vector<double> ranges;
  using RangeType = decltype(ranges)::value_type;
  tf2::Transform pose;  // how the laser is mounted relative to base_link

  LaserData(const sensor_msgs::LaserScan & scan, const tf2::Transform & pose);

  tf2::Quaternion get_angle(std::size_t i) const;
  tf2::Vector3 get_range(std::size_t i) const;
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
  virtual double sensor_update(ParticleFilter * pf, const LaserData & data) = 0;
};
