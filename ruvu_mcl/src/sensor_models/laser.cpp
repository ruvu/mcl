#include "./laser.hpp"

#include "sensor_msgs/LaserScan.h"

LaserData::LaserData(const sensor_msgs::LaserScan & scan, const tf2::Transform & pose)
: angle_min(scan.angle_min), angle_increment(scan.angle_increment), ranges(), pose(pose)
{
  ranges.reserve(scan.ranges.size());
  for (auto & range : scan.ranges) {
    ranges.push_back(range);
  }
}

tf2::Vector3 LaserData::get_range(std::size_t i) const
{
  auto angle = angle_min + i * angle_increment;
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);

  tf2::Vector3 v{ranges[i], 0, 0};
  return tf2::quatRotate(q, v);
}

Laser::~Laser() = default;
