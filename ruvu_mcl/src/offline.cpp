// Copyright 2021 RUVU Robotics B.V.

#include <memory>
#include <string>
#include <vector>

#include "./filter.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/init.h"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/buffer.h"

constexpr auto name = "offline";

ros::Duration duration_from_bag(const rosbag::Bag & bag)
{
  std::vector<ros::Time> ts;
  for (const auto & msg : rosbag::View(bag, rosbag::TopicQuery("/tf"))) {
    tf2_msgs::TFMessageConstPtr tfs = msg.instantiate<tf2_msgs::TFMessage>();
    if (tfs == nullptr) throw std::runtime_error("Received tf message with the wrong type");
    for (const auto & tf : tfs->transforms) ts.push_back(tf.header.stamp);
  }

  ROS_INFO("Loaded %zu tf messages", ts.size());
  if (!ts.size()) throw std::runtime_error("Can't find any scans in the bagfile");

  ROS_INFO("Loaded tf messages, first=%f last=%f", ts.front().toSec(), ts.back().toSec());
  return ts.back() - ts.front();
}

class BagBuffer : public tf2_ros::Buffer
{
public:
  explicit BagBuffer(const rosbag::Bag & bag) : Buffer(duration_from_bag(bag))
  {
    std::vector<std::string> topics = {"/tf", "/tf_static"};
    for (const auto & msg : rosbag::View(bag, rosbag::TopicQuery(topics))) {
      auto tfs = msg.instantiate<tf2_msgs::TFMessage>();
      if (tfs == nullptr) throw std::runtime_error("Received tf message with the wrong type");
      for (auto & tf : tfs->transforms) {
        this->setTransform(tf, "bagfile", msg.getTopic() == "/tf_static");
      }
    }
  }
};

class Offline
{
public:
  Offline(const rosbag::Bag & bag, ros::NodeHandle nh, ros::NodeHandle private_nh)
  : buffer_(std::make_shared<BagBuffer>(bag)), filter_(nh, private_nh, buffer_)
  {
    auto scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
    auto map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    std::vector<std::string> topics = {"/scan", "/map", "/initialpose"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::WallDuration sleep{private_nh.param("sleep", 0.0)};
    for (const auto & msg : view) {
      if (!ros::ok()) break;

      if (sensor_msgs::LaserScanConstPtr scan = msg.instantiate<sensor_msgs::LaserScan>()) {
        scan_pub.publish(scan);
        filter_.scan_cb(scan);
      } else if (nav_msgs::OccupancyGridConstPtr map = msg.instantiate<nav_msgs::OccupancyGrid>()) {
        map_pub.publish(map);
        filter_.map_cb(map);
      } else if (
        geometry_msgs::PoseWithCovarianceStampedConstPtr initialpose =
          msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>()) {
        filter_.initial_pose_cb(initialpose);
      } else {
        ROS_WARN_STREAM("Unsupported message type " << msg.getTopic());
      }
      sleep.sleep();
    }
  }

private:
  std::shared_ptr<BagBuffer> buffer_;
  Filter filter_;
};

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

  rosbag::Bag bag;
  bag.open(args[1], rosbag::bagmode::Read);

  Offline node{bag, nh, private_nh};

  bag.close();
  ROS_INFO_NAMED(name, "%s finished", private_nh.getNamespace().c_str());
  return EXIT_SUCCESS;
}
