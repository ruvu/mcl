// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "ros/message_forward.h"
#include "ros/publisher.h"

class ParticleFilter;
namespace ros
{
class NodeHandle;
}
namespace std_msgs
{
ROS_DECLARE_MESSAGE(Header)
}

class CloudPublisher
{
public:
  CloudPublisher(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void publish(const std_msgs::Header & header, const ParticleFilter & pf);

private:
  ros::Publisher cloud_pub_;
};
