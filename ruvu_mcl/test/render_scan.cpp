// Copyright 2021 RUVU Robotics B.V.

#include <Magick++.h>

#include <iostream>

#include "../src/map.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/topic.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

Magick::Image cells_to_image(const OccupancyMap::CellsType cells)
{
  Magick::Image img;
  img.size(Magick::Geometry{
    static_cast<unsigned int>(cells.rows()), static_cast<unsigned int>(cells.cols())});

  for (std::size_t x = 0; x < cells.rows(); ++x) {
    for (std::size_t y = 0; y < cells.cols(); ++y) {
      auto occ = cells(x, y);
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
  return img;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "render_scan");
  ros::NodeHandle nh;

  OccupancyMap map{*ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map")};

  auto range = map.calc_range(10, 10, 100, 100);
  ROS_INFO_STREAM("range: " << range);

  // Magick::InitializeMagick(nullptr);
  // auto img = cells_to_image(map.cells);
  // img.display();
  return EXIT_SUCCESS;
}
