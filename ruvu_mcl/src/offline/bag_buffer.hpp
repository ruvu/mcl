// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "tf2_ros/buffer.h"

namespace rosbag
{
class Bag;
}

namespace ruvu_mcl
{
/**
 * @brief Load all tf messages from a rosbag into a tf2_ros::Buffer
 */
class BagBuffer : public tf2_ros::Buffer
{
public:
  explicit BagBuffer(const rosbag::Bag & bag);
};
}  // namespace ruvu_mcl
