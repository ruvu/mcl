#include "./map.hpp"

#include "nav_msgs/OccupancyGrid.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Map::Map(const nav_msgs::OccupancyGrid & msg)
{
  scale = msg.info.resolution;

  tf2::fromMsg(msg.info.origin, origin);

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

bool Map::is_valid(int i, int j)
{
  return i >= 0 && i < cells.rows() && j >= 0 && j < cells.cols();
}

double Map::calc_range(int x, int y, int x1, int y1)
{
  auto dx = abs(x1 - x);
  auto sx = x < x1 ? 1 : -1;
  auto dy = -abs(y1 - y);
  auto sy = y < y1 ? 1 : -1;
  auto err = dx + dy; /* error value e_xy */
  while (true) {
    if (!is_valid(x, y) || !cells(x, y) <= 0) {
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

void Map::render(const tf2::Transform & pose) {}
