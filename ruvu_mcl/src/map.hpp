// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "Eigen/Eigen"
#include "ros/message_forward.h"
#include "tf2/LinearMath/Transform.h"

// forward declare
namespace nav_msgs
{
ROS_DECLARE_MESSAGE(OccupancyGrid)
}

struct Map
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  using CellType = int8_t;

  // rows = x, cols = y
  using CellsType = Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

  tf2::Transform origin;

  // Map scale (m/cell)
  double scale;

  CellsType cells;

  explicit Map() {}
  explicit Map(const nav_msgs::OccupancyGrid & msg);

  bool is_valid(int i, int j) const;
  double calc_range(const tf2::Vector3 & v1, const tf2::Vector3 & v2) const;
  double calc_range(int x0, int y0, int x1, int y1) const;
  std::pair<int, int> world2map(const tf2::Vector3 & v) const;
};
