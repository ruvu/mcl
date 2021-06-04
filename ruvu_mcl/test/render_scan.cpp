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
      Magick::Color color;
      if (occ < 0) {  // free
        color = Magick::ColorGray(254 / 255.0);
      } else if (occ == 0) {  // unknown
        color = Magick::ColorGray(205 / 255.0);
      } else {  // occ
        color = Magick::ColorGray(0 / 255.0);
      }
      img.pixelColor(x, y, color);
    }
  }

  img.display();
}
