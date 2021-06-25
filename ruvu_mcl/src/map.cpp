// Copyright 2021 RUVU Robotics B.V.

#include "./map.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <utility>

#include "nav_msgs/OccupancyGrid.h"
#include "ros/console.h"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

constexpr auto name = "map";

Map::Map(const nav_msgs::OccupancyGrid & msg)
{
  scale = msg.info.resolution;

  tf2::fromMsg(msg.info.origin, origin);
  origin = origin.inverse();

  if (msg.info.width * msg.info.height != msg.data.size())
    throw std::runtime_error("msg.info.width * msg.info.height != msg.data.size()");

  ROS_INFO_NAMED(
    name, "Converted a %d X %d map @ %.3f m/pix (%f MB)", msg.info.width, msg.info.height,
    msg.info.resolution, msg.data.size() / 1024.0 / 1024.0);
}

Map::operator nav_msgs::OccupancyGrid()
{
  nav_msgs::OccupancyGrid msg;
  msg.info.resolution = scale;

  tf2::toMsg(origin.inverse(), msg.info.origin);
  return msg;
}

template <typename T>
int floor2int(T v)
{
  return static_cast<int>(floor(v));
}

std::pair<int, int> Map::world2map(const tf2::Vector3 & v) const
{
  tf2::Vector3 w = origin * v;
  int i = floor2int(w.getX() / scale + 0.5);
  int j = floor2int(w.getY() / scale + 0.5);
  return {i, j};
}

OccupancyMap::OccupancyMap(const nav_msgs::OccupancyGrid & msg) : Map(msg)
{
  /**
   * copy the data from the message
   * An OccupancyGrid is stored in row-major order, where the first axis is the x axis. This means
   * that you should access it as `i + width * j`.
   * We want to access the map later as cells(i, j).
   * This means rows == x == width, cols == y == height
   *
   *   y (cols, height) ->
   * x 0 3
   *   1 4
   *   2 5
   *
   * In Eigen this storage order is called col-major
   */
  cells = Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>(
            msg.data.data(), msg.info.width, msg.info.height)
            .unaryExpr([](int8_t cell) -> CellType {
              if (cell == 0) {
                return -1;
              } else if (cell == 100) {
                return +1;
              } else {
                return 0;
              }
            });
}

bool OccupancyMap::is_valid(int i, int j) const
{
  return i >= 0 && i < cells.rows() && j >= 0 && j < cells.cols();
}

double OccupancyMap::calc_range(const tf2::Vector3 & v1, const tf2::Vector3 & v2) const
{
  auto [i0, j0] = world2map(v1);
  auto [i1, j1] = world2map(v2);
  return calc_range(i0, j0, i1, j1) * scale;
}

double OccupancyMap::calc_range(int x0, int y0, int x1, int y1) const
{
  auto x = x0, y = y0;
  auto dx = abs(x1 - x);
  auto dy = -abs(y1 - y);
  auto sx = x < x1 ? 1 : -1;
  auto sy = y < y1 ? 1 : -1;
  auto err = dx + dy; /* error value e_xy */
  while (true) {
    if (!is_valid(x, y) || !(cells(x, y) <= 0)) {
      return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));
    }

    if (x == x1 && y == y1) break;
    auto e2 = 2 * err;
    if (e2 >= dy) {  // e_xy + e_x > 0
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {  // e_xy + e_y < 0
      err += dx;
      y += sy;
    }
  }
  return std::numeric_limits<double>::infinity();
}

using Coordinate = std::pair<Eigen::Index, Eigen::Index>;
// using QueueData = std::pair<CellType, Coordinate>;
struct QueueData
{
  double distance;
  Coordinate cell;
  Coordinate source;  // original obstacle location, for calculating distances
};

bool operator<(const QueueData & a, const QueueData & b)
{
  return a.distance > b.distance;  // lower prio is earlier
}

DistanceMap::DistanceMap(const nav_msgs::OccupancyGrid & msg) : Map(msg)
{
  // during unittests, skip ros publishing
  if (ros::isInitialized()) {
    ros::NodeHandle nh("~");
    debug_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("distance_map", 1, true);
  }

  // TODO(Ramon): move common parsing functionality to a utility function
  auto obstacles =
    Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>(
      msg.data.data(), msg.info.width, msg.info.height)
      .unaryExpr([](int8_t cell) -> int8_t { return cell == 100; });
  cells.setConstant(obstacles.rows(), obstacles.cols(), -std::numeric_limits<double>::infinity());

  std::priority_queue<QueueData> q;
  {
    // queues all occupied cells to the priority queue
    struct QueueObstacles
    {
      explicit QueueObstacles(std::priority_queue<QueueData> * q) : q_(q) {}
      std::priority_queue<QueueData> * q_;
      void init(const int8_t & value, Eigen::Index i, Eigen::Index j) { operator()(value, i, j); }
      void operator()(const int8_t & value, Eigen::Index i, Eigen::Index j)
      {
        if (value) {
          q_->push(QueueData{0, {i, j}, {i, j}});
        }
      }
    } v{&q};
    obstacles.visit(v);
  }
  {
    auto q2 = q;  // copy
    while (!q2.empty()) {
      auto d = q2.top();
      q2.pop();
      auto [i, j] = d.cell;
      cells(i, j) = 0;  // mark as completed
    }
  }

  auto enqueue = [&](const Coordinate & source, const Coordinate & cell) {
    auto [i, j] = cell;
    if (i < 0 || j < 0 || i >= cells.rows() || j >= cells.cols()) return;

    // if the cell is already marked, the distance is always lower, so skip
    if (cells(i, j) >= 0) {
      // ROS_DEBUG_NAMED(name, "skipping already marked %zi %zi", i, j);
      return;
    }

    auto [k, l] = source;
    auto di = i - k;
    auto dj = j - l;

    double distance = cells(i, j) = sqrt(di * di + dj * dj) * scale;
    // ROS_DEBUG_NAMED(name, "marking %zi %zi with %f", i, j, distance);
    q.push(QueueData{distance, {i, j}, {k, l}});
  };

  while (!q.empty()) {
    auto d = q.top();
    q.pop();

    auto [i, j] = d.cell;
    // ROS_DEBUG_NAMED(name, "processing %zi %zi of distance %f", i, j, d.distance);
    enqueue(d.source, {i - 1, j});
    enqueue(d.source, {i, j - 1});
    enqueue(d.source, {i + 1, j});
    enqueue(d.source, {i, j + 1});
  }

  if (debug_pub_.getNumSubscribers())
    debug_pub_.publish(static_cast<nav_msgs::OccupancyGrid>(*this));
}

DistanceMap::operator nav_msgs::OccupancyGrid()
{
  auto msg = Map::operator nav_msgs::OccupancyGrid();
  msg.data.resize(cells.size());

  Eigen::Map<Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>(
    msg.data.data(), cells.rows(), cells.cols()) =
    cells.unaryExpr([](auto cell) -> int8_t { return std::max(100 - 100 * cell, 0.); });
  msg.info.width = cells.rows();
  msg.info.height = cells.cols();
  return msg;
}

double DistanceMap::closest_obstacle(const tf2::Vector3 & v) const
{
  auto [i, j] = world2map(v);
  if (!is_valid(i, j)) {
    return std::numeric_limits<double>::infinity();
  } else {
    return cells(i, j);
  }
}

bool DistanceMap::is_valid(int i, int j) const
{
  return i >= 0 && i < cells.rows() && j >= 0 && j < cells.cols();
}
