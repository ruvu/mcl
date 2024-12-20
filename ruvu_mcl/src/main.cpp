// Copyright 2021 RUVU Robotics B.V.

#include "./node.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"

using ruvu_mcl::Node;

constexpr auto name = "main";

/**
* @brief Main ros node of the ruvu_mcl package
*/
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "mcl");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  Node node{nh, private_nh};
  ROS_INFO_NAMED(name, "%s started", private_nh.getNamespace().c_str());
  ros::spin();
  return EXIT_SUCCESS;
}
