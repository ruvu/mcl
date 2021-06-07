#include "./map.hpp"

#include "nav_msgs/OccupancyGrid.h"
#include "ros/console.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Map::Map(const nav_msgs::OccupancyGrid & msg)
{
  scale = msg.info.resolution;

  tf2::fromMsg(msg.info.origin, origin);

  if (msg.info.width * msg.info.height != msg.data.size())
    throw std::runtime_error("msg.info.width * msg.info.height != msg.data.size()");

  // copy data
  cells =
    Eigen::Map<const Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      msg.data.data(), msg.info.width, msg.info.height)
      .unaryExpr([](CellType cell) -> CellType {
        if (cell == 0) {
          return -1;
        } else if (cell == 100) {
          return +1;
        } else {
          return 0;
        }
      });

  // modify what the values mean
}

bool Map::is_valid(int i, int j) const
{
  return i >= 0 && i < cells.rows() && j >= 0 && j < cells.cols();
}

double Map::calc_range(const tf2::Vector3 & v1, const tf2::Vector3 & v2) const
{
  ROS_INFO("calc_range from %f %f to %f %f", v1.getX(), v1.getY(), v2.getX(), v2.getY());
  auto [i0, j0] = world2map(v1);
  auto [i1, j1] = world2map(v2);
  return calc_range(i0, j0, i1, j1) * scale;
}

double Map::calc_range(int x, int y, int x1, int y1) const
{
  ROS_INFO("calc_range from %i %i to %i %i", x, y, x1, y1);
  auto dx = abs(x1 - x);
  auto sx = x < x1 ? 1 : -1;
  auto dy = -abs(y1 - y);
  auto sy = y < y1 ? 1 : -1;
  auto err = dx + dy; /* error value e_xy */
  while (true) {
    if (is_valid(x, y)) {
      ROS_INFO("checking cell %ix%i = %i", x, y, cells(x, y));
    } else {
      ROS_INFO("cell not valid");
    }

    if (!is_valid(x, y) || !(cells(x, y) <= 0)) {
      return sqrt(x * x + y * y);
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

template <typename T>
int floor2int(T v)
{
  ROS_INFO("floor: %f", v);
  return static_cast<int>(floor(v));
}

std::pair<int, int> Map::world2map(const tf2::Vector3 & v) const
{
  tf2::Vector3 w = origin.inverse() * v;
  int i = floor2int(w.getX() / scale + 0.5);
  int j = floor2int(w.getY() / scale + 0.5);
  ROS_INFO("world2map: %f %f -> %f %f -> %i %i", v.getX(), v.getY(), w.getX(), w.getY(), i, j);
  return {i, j};
}
