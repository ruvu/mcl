// Copyright 2021 RUVU Robotics B.V.

#include "./laser.hpp"

#include <limits>

#include "sensor_msgs/LaserScan.h"

namespace ruvu_mcl
{
LaserData::LaserData(const sensor_msgs::LaserScan & scan, const tf2::Transform & pose)
: header(scan.header),
  pose(pose),
  angle_min(scan.angle_min),
  angle_increment(scan.angle_increment),
  range_max(scan.range_max),
  ranges()
{
  ranges.reserve(scan.ranges.size());
  for (const auto & range : scan.ranges) {
    // https://www.ros.org/reps/rep-0117.html

    // Represents expected pre-REP logic and is the only necessary condition for most applications.
    if (scan.range_min <= range && range <= scan.range_max) {
      // This is a valid measurement.
      ranges.push_back(range);
    } else if (!isfinite(range) && range < 0) {
      // Object too close to measure.
      ranges.push_back(-std::numeric_limits<RangeType>::infinity());
    } else if (!isfinite(range) && range > 0) {
      // No objects detected in range.
      ranges.push_back(std::numeric_limits<RangeType>::infinity());
    } else if (isnan(range)) {
      // This is an erroneous, invalid, or missing measurement.
      ranges.push_back(range);
    } else {
      // The sensor reported these measurements as valid, but they are discarded per the limits
      // defined by minimum_range and maximum_range.
      ranges.push_back(std::numeric_limits<RangeType>::quiet_NaN());
    }
  }
}

double LaserData::get_angle(std::size_t i) const { return angle_min + i * angle_increment; }

Laser::~Laser() = default;
}  // namespace ruvu_mcl
