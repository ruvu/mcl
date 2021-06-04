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
  using CellType = int8_t;
  using CellsType = Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic>;

  tf2::Transform origin;

  // Map scale (m/cell)
  double scale;

  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  CellsType cells;

  explicit Map() {}
  explicit Map(const nav_msgs::OccupancyGrid & msg);

  bool is_valid(int i, int j);
  double calc_range(int x0, int y0, int x1, int y1);
  void render(const tf2::Transform & pose);
};
