// Copyright 2021 RUVU Robotics B.V.

#include "./cloud_publisher.hpp"

#include <algorithm>
#include <utility>

#include "./particle_filter.hpp"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

CloudPublisher::CloudPublisher(ros::NodeHandle nh, ros::NodeHandle private_nh)
: cloud_pub_(private_nh.advertise<visualization_msgs::Marker>("cloud", 1))
{
}

void CloudPublisher::publish(const std_msgs::Header & header, const ParticleFilter & pf)
{
  constexpr double length = 0.1;
  constexpr double width = 0.02;

  visualization_msgs::Marker m;
  m.header = header;
  m.ns = "particles";
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::MODIFY;
  m.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
  m.scale.x = width;
  m.points.reserve(pf.particles.size() * 2);
  m.colors.reserve(pf.particles.size() * 2);

  double max_weight = 0;
  for (const auto & p : pf.particles) max_weight = std::max(max_weight, p.weight);

  tf2::Vector3 p1{length / 2, 0, 0};
  for (const auto & particle : pf.particles) {
    std_msgs::ColorRGBA c;
    c.a = particle.weight / max_weight;
    c.b = 1;
    m.colors.push_back(c);
    m.colors.push_back(std::move(c));
    {
      geometry_msgs::Point tmp;
      tf2::toMsg(particle.pose * p1, tmp);
      m.points.push_back(std::move(tmp));
    }
    {
      geometry_msgs::Point tmp;
      tf2::toMsg(particle.pose * -p1, tmp);
      m.points.push_back(std::move(tmp));
    }
  }

  cloud_pub_.publish(std::move(m));
}
