// Copyright 2021 RUVU Robotics B.V.

#include "./bag_buffer.hpp"

#include <string>
#include <vector>

#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "tf2_msgs/TFMessage.h"

namespace ruvu_mcl
{
ros::Duration duration_from_bag(const rosbag::Bag & bag)
{
  std::vector<ros::Time> ts;
  for (const auto & msg : rosbag::View(bag, rosbag::TopicQuery("/tf"))) {
    tf2_msgs::TFMessageConstPtr tfs = msg.instantiate<tf2_msgs::TFMessage>();
    if (tfs == nullptr) throw std::runtime_error("Received tf message with the wrong type");
    for (const auto & tf : tfs->transforms) ts.push_back(tf.header.stamp);
  }

  ROS_INFO("Loaded %zu tf messages", ts.size());
  if (ts.empty()) throw std::runtime_error("Can't find any scans in the bagfile");

  ROS_INFO("Loaded tf messages, first=%f last=%f", ts.front().toSec(), ts.back().toSec());
  return ts.back() - ts.front();
}

BagBuffer::BagBuffer(const rosbag::Bag & bag) : Buffer(duration_from_bag(bag))
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
}  // namespace ruvu_mcl
