#include <Magick++.h>

#include <iostream>

#include "../src/map.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/topic.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "render_scan");

  Map map{*ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map")};

  Magick::InitializeMagick(nullptr);

  Magick::Image img;
  img.size(Magick::Geometry{map.cells.rows(), map.cells.cols()});

  for (std::size_t x = 0; x < map.cells.rows(); ++x) {
    for (std::size_t y = 0; y < map.cells.cols(); ++y) {
      auto occ = map.cells(x, y);
      img.pixelColor(x, y, Magick::ColorGray{occ});
    }
  }

  img.display();
}
