// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "Eigen/Core"
#include "ros/message_forward.h"
#include "ros/publisher.h"
#include "tf2/LinearMath/Transform.h"

// forward declare
namespace nav_msgs
{
ROS_DECLARE_MESSAGE(OccupancyGrid)
}

namespace ruvu_mcl
{
/**
 * @brief Base class for 2-D grid maps
 */
struct Map
{
  /**
   * @brief The origin of the map
   */
  tf2::Transform origin;

  /**
   * @brief Map scale (m/cell)
   */
  double scale;

  /**
   * @brief Construct a Map from a nav_msgs::OccupancyGrid
   */
  explicit Map(const nav_msgs::OccupancyGrid & msg);

  /**
   * @brief Convert a Map to a nav_msgs::OccupancyGrid
   */
  explicit operator nav_msgs::OccupancyGrid() const;

  /**
   * @brief Convert world coordinates to map indices
   */
  std::pair<Eigen::Index, Eigen::Index> world2map(const tf2::Vector3 & v) const;
};

/**
 * @brief A 2-D grid map, in which each cell has a occupance state
 */
struct OccupancyMap : Map
{
  /**
   * @brief A cell contains the occupancy state (-1 = free, 0 = unknown, +1 = occ)
   */
  using CellType = int8_t;

  using CellsType = Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

  CellsType cells;

  /**
   * @brief Construct a Map from a nav_msgs::OccupancyGrid
   */
  explicit OccupancyMap(const nav_msgs::OccupancyGrid & msg);

  /**
   * @brief Check if the cells in the line between v1 and v2 are occupied
   * @param v1 from
   * @param v2 to
   * @return if the range is occupied
   */
  double calc_range(const tf2::Vector3 & v1, const tf2::Vector3 & v2) const;

  /**
   * @brief Check if the cells in the line between x0,y0 and x1,y1 are occupied
   * @return if the range is occupied
   */
  double calc_range(Eigen::Index x0, Eigen::Index y0, Eigen::Index x1, Eigen::Index y1) const;

  /**
   * @brief Check if the indices are in bounds of the map
   */
  bool is_valid(Eigen::Index i, Eigen::Index j) const;
};

/**
 * @brief A 2-D grid map, in which each cell has the distance to the nearest obstacle
 */
struct DistanceMap : Map
{
  /**
   * @brief A cell contains the distance to the nearest obstacle
   */
  using CellType = double;

  using CellsType = Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

  CellsType cells;
  ros::Publisher debug_pub_;

  /**
   * @brief Construct a DistanceMap from a nav_msgs::OccupancyGrid
   */
  explicit DistanceMap(const nav_msgs::OccupancyGrid & msg);

  /**
   * @brief Convert a DistanceMap to a nav_msgs::OccupancyGrid
   */
  explicit operator nav_msgs::OccupancyGrid() const;

  /**
   * @brief Get the distance to the closest obstacle from a given point
   */
  double closest_obstacle(const tf2::Vector3 & v) const;

  /**
   * @brief Check if the indices are in bounds of the map
   */
  bool is_valid(Eigen::Index i, Eigen::Index j) const;  // TODO(Ramon): move to baseclass
};
}  // namespace ruvu_mcl
