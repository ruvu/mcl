#include "./laser.hpp"

#include "sensor_msgs/LaserScan.h"

LaserData::LaserData(const sensor_msgs::LaserScan & scan, const tf2::Transform & pose)
: angle_min(scan.angle_min),
  angle_increment(scan.angle_increment),
  range_max(scan.range_max),
  ranges(),
  pose(pose)
{
  ranges.reserve(scan.ranges.size());
  for (auto & range : scan.ranges) {
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
      // The sensor reported these measurements as valid, but they are discarded per the limits defined by minimum_range and maximum_range.
      ranges.push_back(std::numeric_limits<RangeType>::quiet_NaN());
    }
  }
}

tf2::Quaternion LaserData::get_angle(std::size_t i) const
{
  auto angle = angle_min + i * angle_increment;
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  return q;
}

tf2::Vector3 LaserData::get_range(std::size_t i) const
{
  tf2::Vector3 v{ranges[i], 0, 0};
  return tf2::quatRotate(get_angle(i), v);
}

Laser::~Laser() = default;
