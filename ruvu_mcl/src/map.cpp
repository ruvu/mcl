#include "./map.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Map::Map(const nav_msgs::OccupancyGrid & msg)
{
  scale = msg.info.resolution;

  tf2::fromMsg(msg.info.origin, origin);

  // copy data
  cells =
    Eigen::Map<const Eigen::Matrix<CellType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      msg.data.data(), msg.info.width, msg.info.height);
}

void Map::render(const tf2::Transform & pose) {}
