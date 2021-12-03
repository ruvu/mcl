// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <vector>

#include "ros/message_forward.h"
#include "ros/publisher.h"

namespace ros
{
class Particle;
}
namespace std_msgs
{
ROS_DECLARE_MESSAGE(Header)
}

namespace ruvu_mcl
{
class Particle;

/**
 * @brief Publishes a vector of particles as a visualization_msgs::Marker
 */
class CloudPublisher
{
public:
  CloudPublisher(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void publish(const std_msgs::Header & header, const std::vector<Particle> & pf);

private:
  ros::Publisher cloud_pub_;
};
}  // namespace ruvu_mcl
