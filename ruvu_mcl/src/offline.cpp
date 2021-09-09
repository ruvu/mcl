// Copyright 2021 RUVU Robotics B.V.

#include <memory>
#include <string>
#include <vector>

#include "./filter.hpp"
#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/init.h"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "ruvu_mcl/AMCLConfig.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
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
    dynamic_reconfigure::Server<ruvu_mcl::AMCLConfig> reconfigure_server;
    reconfigure_server.setCallback([this](const ruvu_mcl::AMCLConfig & config, uint32_t level) {
      ROS_INFO_NAMED(name, "reconfigure call");
      filter_.configure(Config{config});
    });

    auto scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 100);
    auto landmark_pub = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmarks", 100);
    auto map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    auto landmark_list_pub = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmark_list", 1, true);
    auto tf_pub = nh.advertise<tf2_msgs::TFMessage>("/tf", 100);
    auto tf_static_pub = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);

    ros::WallDuration{0.1}.sleep();  // wait for topics to connect

    std::vector<std::string> topics = {"/scan",        "/map", "/landmarks", "/landmark_list",
                                       "/initialpose", "/tf",  "/tf_static"};
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
        ruvu_mcl_msgs::LandmarkListConstPtr landmarks =
          msg.instantiate<ruvu_mcl_msgs::LandmarkList>()) {
        if (msg.getTopic() == "/landmarks") {
          landmark_pub.publish(landmarks);
          filter_.landmark_cb(landmarks);
        } else if (msg.getTopic() == "/landmark_list") {
          landmark_list_pub.publish(landmarks);
          filter_.landmark_list_cb(landmarks);
        } else {
          ROS_WARN_STREAM("Unsupported message type " << msg.getTopic());
        }
      } else if (
        geometry_msgs::PoseWithCovarianceStampedConstPtr initialpose =
          msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>()) {
        filter_.initial_pose_cb(initialpose);
      } else if (tf2_msgs::TFMessageConstPtr tf = msg.instantiate<tf2_msgs::TFMessage>()) {
        if (msg.getTopic() == "/tf") {
          tf_pub.publish(tf);
        } else if (msg.getTopic() == "/tf_static") {
          tf_static_pub.publish(tf);
        } else {
          ROS_WARN_STREAM("Unsupported message type " << msg.getTopic());
        }
      } else {
        ROS_WARN_STREAM("Unsupported message type " << msg.getTopic());
      }
      sleep.sleep();
      ros::spinOnce();  // handle dynamic reconfigure calls
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
