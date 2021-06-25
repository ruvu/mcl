// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "Eigen/Eigen"
#include "ros/message_forward.h"
#include "ros/publisher.h"
#include "tf2/LinearMath/Transform.h"

// forward declare
namespace nav_msgs
{
ROS_DECLARE_MESSAGE(OccupancyGrid)
}

struct Map
{
  tf2::Transform origin;

  // Map scale (m/cell)
  double scale;

  explicit Map(const nav_msgs::OccupancyGrid & msg);
  operator nav_msgs::OccupancyGrid();

  std::pair<int, int> world2map(const tf2::Vector3 & v) const;
};

struct OccupancyMap : Map
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  using CellType = int8_t;

  // rows = x, cols = y
  using CellsType = Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

  CellsType cells;

  explicit OccupancyMap(const nav_msgs::OccupancyGrid & msg);

  double calc_range(const tf2::Vector3 & v1, const tf2::Vector3 & v2) const;
  double calc_range(int x0, int y0, int x1, int y1) const;
  bool is_valid(int i, int j) const;
};

struct DistanceMap : Map
{
  // distance to the nearest obstacle
  using CellType = double;

  // rows = x, cols = y
  using CellsType = Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

  CellsType cells;
  ros::Publisher debug_pub_;

  explicit DistanceMap(const nav_msgs::OccupancyGrid & msg);
  operator nav_msgs::OccupancyGrid();

  double closest_obstacle(const tf2::Vector3 & v) const;
  bool is_valid(int i, int j) const;  // TODO(Ramon): move to baseclass
};
