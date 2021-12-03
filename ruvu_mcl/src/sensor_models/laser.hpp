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

/**
 * @brief Measurment data from a laser scanner
 */
class LaserData
{
public:
  using RangeType = double;

  /**
   * @brief Construct a LaserData from a ROS message
   * @param scan ROS message
   * @param pose The position of the scanner relative to base_link
   */
  LaserData(const sensor_msgs::LaserScan & scan, const tf2::Transform & pose);

  /**
   * @brief Calculate the angle of the i'th beam
   */
  double get_angle(std::size_t i) const;

  /**
   * @brief timestamp in the header is the acquisition time of the first ray in the scan.
   */
  std_msgs::Header header;
  /**
   * @brief how the laser is mounted relative to base_link
   */
  tf2::Transform pose;

  /**
   * @brief start angle of the scan
   */
  double angle_min;

  /**
   * @brief angular distance between measurements [rad]
   */
  double angle_increment;

  /**
   * @brief maximum range value [m]
   */
  double range_max;

  /**
   * @brief range data [m]
   *
   * If a range is -infinity, the object too close to measure.
   * If a range is infinity, no objects are detected in range.
   * If the range is nan, it is an erroneous, invalid, or missing measurement
   */
  std::vector<RangeType> ranges;
};

/**
 * @brief Base class for laser sensor models
 *
 * Base class for all laser sensor models. A sensor model tries to accurately model the specific
 * types of uncertainty that exist in a sensor.
 *
 * Sensor models take a measurment as input and update the weight of the particles according to how
 * good the position estimate explains the sensor measurment. See also chapter 6 of Probabilistc
 * Robotics.
 */
class Laser
{
public:
  virtual ~Laser();

  /**
   * @brief Update particles based on a laser measurement
   * @param pf The particle filter to update
   * @param data Laser data to use
   */
  virtual void sensor_update(ParticleFilter * pf, const LaserData & data) = 0;
};
}  // namespace ruvu_mcl
