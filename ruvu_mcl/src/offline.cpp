// Copyright 2021 RUVU Robotics B.V.

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "./filter.hpp"
#include "./offline/bag_buffer.hpp"
#include "./offline/bag_player.hpp"
#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ruvu_mcl/AMCLConfig.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_msgs/TFMessage.h"

constexpr auto name = "offline";

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "mcl");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  ROS_INFO_NAMED(name, "%s started", private_nh.getNamespace().c_str());

  std::vector<std::string> args;
  ros::removeROSArgs(argc, argv, args);
  if (args.size() != 2) {
    puts("usage: offline BAGFILE");
    exit(EXIT_FAILURE);
  }

  BagPlayer player{args[1]};
  player.set_playback_speed(private_nh.param("rate", std::numeric_limits<double>::infinity()));

  auto buffer = std::make_shared<BagBuffer>(player.bag);
  Filter filter{nh, private_nh, buffer};

  dynamic_reconfigure::Server<ruvu_mcl::AMCLConfig> reconfigure_server;
  reconfigure_server.setCallback([&filter](const ruvu_mcl::AMCLConfig & config, uint32_t level) {
    ROS_INFO_NAMED(name, "reconfigure call");
    filter.configure(Config{config});
  });

  auto scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 100);
  auto landmark_pub = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmarks", 100);
  auto map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  auto landmark_list_pub = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmark_list", 1, true);
  auto tf_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 100);
  auto tf_static_pub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);

  ros::WallDuration{0.1}.sleep();  // wait for topics to connect

  player.register_callback<sensor_msgs::LaserScan>(
    "/scan", [&filter, &scan_pub](const sensor_msgs::LaserScanConstPtr & scan) {
      scan_pub.publish(scan);
      filter.scan_cb(scan);
    });

  player.register_callback<ruvu_mcl_msgs::LandmarkList>(
    "/landmarks", [&filter, &landmark_pub](const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks) {
      landmark_pub.publish(landmarks);
      filter.landmark_cb(landmarks);
    });

  player.register_callback<nav_msgs::OccupancyGrid>(
    "/map", [&filter, &map_pub](const nav_msgs::OccupancyGridConstPtr & map) {
      map_pub.publish(map);
      filter.map_cb(map);
    });

  player.register_callback<ruvu_mcl_msgs::LandmarkList>(
    "/landmark_list",
    [&filter, &landmark_list_pub](const ruvu_mcl_msgs::LandmarkListConstPtr & landmark_list) {
      landmark_list_pub.publish(landmark_list);
      filter.landmark_list_cb(landmark_list);
    });

  player.register_callback<geometry_msgs::PoseWithCovarianceStamped>(
    "/initialpose",
    [&filter](const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose) {
      filter.initial_pose_cb(initial_pose);
    });

  player.register_callback<tf2_msgs::TFMessage>(
    "/tf", [&filter, &tf_pub](const tf2_msgs::TFMessageConstPtr & tf) { tf_pub.publish(tf); });

  player.register_callback<tf2_msgs::TFMessage>(
    "/tf_static", [&filter, &tf_static_pub](const tf2_msgs::TFMessageConstPtr & tf) {
      tf_static_pub.publish(tf);
    });

  player.start_play();

  ROS_INFO_NAMED(name, "%s finished", private_nh.getNamespace().c_str());
  return EXIT_SUCCESS;
}
