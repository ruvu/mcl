// Copyright 2021 RUVU Robotics B.V.

#include "./node.hpp"

#include <memory>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

constexpr auto name = "node";

Node::Node(ros::NodeHandle nh, ros::NodeHandle private_nh)
: buffer_(std::make_shared<tf2_ros::Buffer>()),
  tf_listener_(*buffer_),
  laser_scan_sub_(nh, "scan", 100),
  laser_scan_filter_(laser_scan_sub_, *buffer_, "", 100, nh),
  landmark_sub_(nh, "landmarks", 100),
  landmark_filter_(landmark_sub_, *buffer_, "", 100, nh),
  map_sub_(nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Node::map_cb, this)),
  landmark_list_sub_(
    nh.subscribe<ruvu_mcl_msgs::LandmarkList>("landmark_list", 1, &Node::landmark_list_cb, this)),
  initial_pose_sub_(nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    "initialpose", 1, &Node::initial_pose_cb, this)),
  reconfigure_server_(private_nh),
  filter_(nh, private_nh, buffer_)
{
  laser_scan_filter_.registerCallback(
    boost::bind(&Node::scan_cb, this, _1, laser_scan_sub_.getTopic()));
  landmark_filter_.registerCallback(
    boost::bind(&Node::landmark_cb, this, _1, landmark_sub_.getTopic()));
  reconfigure_server_.setCallback([this](const ruvu_mcl::AMCLConfig & config, uint32_t level) {
    ROS_INFO_NAMED(name, "reconfigure call");
    Config c{config};
    laser_scan_filter_.setTargetFrame(c.odom_frame_id);
    landmark_filter_.setTargetFrame(c.odom_frame_id);
    filter_.configure(c);
  });
}

Node::~Node() = default;

void Node::scan_cb(
  const sensor_msgs::LaserScanConstPtr & scan, const std::string & sensor_topic_name)
{
  filter_.scan_cb(scan, sensor_topic_name);
}

void Node::landmark_cb(
  const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks, const std::string & sensor_topic_name)
{
  filter_.landmark_cb(landmarks, sensor_topic_name);
}

void Node::map_cb(const nav_msgs::OccupancyGridConstPtr & map) { filter_.map_cb(map); }

void Node::landmark_list_cb(const ruvu_mcl_msgs::LandmarkListConstPtr & landmark_list)
{
  filter_.landmark_list_cb(landmark_list);
}

void Node::initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose)
{
  filter_.initial_pose_cb(initial_pose);
}
