#pragma once

#include "Eigen/Eigen"
#include "nav_msgs/OccupancyGrid.h"
#include "tf2/LinearMath/Transform.h"

struct Map
{
  using CellType = int8_t;
  using CellsType = Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic>;

  tf2::Transform origin;

  // Map scale (m/cell)
  double scale;

  CellsType cells;

  explicit Map() {}
  explicit Map(const nav_msgs::OccupancyGrid & msg);

  void render(const tf2::Transform & pose);
};
