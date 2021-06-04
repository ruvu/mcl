#include "./node.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "mcl");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  Node node{nh, private_nh};
  ROS_INFO("%s started", private_nh.getNamespace().c_str());
  ros::spin();
  return EXIT_SUCCESS;
}
