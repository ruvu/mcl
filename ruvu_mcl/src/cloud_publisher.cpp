// Copyright 2021 RUVU Robotics B.V.

#include "./cloud_publisher.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include "./particle_filter.hpp"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

namespace ruvu_mcl
{
CloudPublisher::CloudPublisher(ros::NodeHandle nh, ros::NodeHandle private_nh)
: cloud_pub_(private_nh.advertise<visualization_msgs::Marker>("cloud", 1))
{
}

void CloudPublisher::publish(const std_msgs::Header & header, const std::vector<Particle> & pf)
{
  constexpr double length = 0.1;
  constexpr double width = 0.02;

  marker_.header = header;
  marker_.ns = "particles";
  marker_.id = 0;
  marker_.type = visualization_msgs::Marker::LINE_LIST;
  marker_.action = visualization_msgs::Marker::MODIFY;
  marker_.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
  marker_.scale.x = width;
  marker_.points.clear();
  marker_.colors.clear();
  marker_.points.reserve(pf.size() * 2);
  marker_.colors.reserve(pf.size() * 2);

  double max_weight = 0;
  for (const auto & p : pf) max_weight = std::max(max_weight, p.weight);

  tf2::Vector3 p1{length / 2, 0, 0};
  for (const auto & particle : pf) {
    std_msgs::ColorRGBA c;
    c.a = particle.weight / max_weight;
    c.b = 1;
    marker_.colors.push_back(c);
    marker_.colors.push_back(c);

    geometry_msgs::Point front;
    tf2::toMsg(particle.pose * p1, front);
    marker_.points.push_back(front);

    geometry_msgs::Point back;
    tf2::toMsg(particle.pose * -p1, back);
    marker_.points.push_back(back);
  }

  cloud_pub_.publish(marker_);
}
}  // namespace ruvu_mcl
