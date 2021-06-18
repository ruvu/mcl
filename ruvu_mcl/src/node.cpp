// Copyright 2021 RUVU Robotics B.V.

#include "./node.hpp"

#include <memory>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

constexpr auto name = "node";

Node::Node(ros::NodeHandle nh, ros::NodeHandle private_nh)
: buffer_(std::make_shared<tf2_ros::Buffer>()),
  tf_listener_(*buffer_),
  laser_scan_sub_(nh, "scan", 100),
  laser_scan_filter_(laser_scan_sub_, *buffer_, "", 100, nh),
  map_sub_(nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Node::map_cb, this)),
  initial_pose_sub_(nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    "initialpose", 1, &Node::initial_pose_cb, this)),
  reconfigure_server_(private_nh),
  filter_(nh, private_nh, buffer_)
{
  laser_scan_filter_.registerCallback(&Node::scan_cb, this);
  reconfigure_server_.setCallback([this](const ruvu_mcl::AMCLConfig & config, uint32_t level) {
    ROS_INFO_NAMED(name, "reconfigure call");
    Config c{config};
    laser_scan_filter_.setTargetFrame(c.odom_frame_id);
    filter_.configure(c);
  });
}

Node::~Node() = default;

void Node::scan_cb(const sensor_msgs::LaserScanConstPtr & scan) { filter_.scan_cb(scan); }

void Node::map_cb(const nav_msgs::OccupancyGridConstPtr & map) { filter_.map_cb(map); }

void Node::initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose)
{
  filter_.initial_pose_cb(initial_pose);
}
