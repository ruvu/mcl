// Copyright 2021 RUVU Robotics B.V.

#include "./likelihood_field_model.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "../map.hpp"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

LikelihoodFieldModel::LikelihoodFieldModel(
  const LikelihoodFieldModelConfig & config, const std::shared_ptr<const DistanceMap> & map)
: config_(config), map_(map)
{
  assert(config_.z_hit + config_.z_rand <= 1.0);
  ros::NodeHandle nh("~");
  debug_pub_ = nh.advertise<visualization_msgs::Marker>("likelihood_field_Model", 1);
}

double LikelihoodFieldModel::sensor_update(ParticleFilter * pf, const LaserData & data)
{
  // Likelihood field range finder model (Page 143 Probabilistc Robotics)

  visualization_msgs::Marker marker;
  marker.header.frame_id = config_.global_frame_id;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  tf2::toMsg(tf2::Transform::getIdentity(), marker.pose);
  marker.scale.x = 0.01;

  bool first = true;  // publish debug info for the first particle
  double total_weight = 0.0;
  auto step = (data.ranges.size() - 1) / (config_.max_beams - 1);
  for (auto & particle : pf->particles) {
    double p = 1.0;

    for (std::size_t i = 0; i < data.ranges.size(); i += step) {
      double obs_range = data.ranges[i];
      auto a = data.get_angle(i);

      // This model ignores max range readings
      if (std::isinf(obs_range)) continue;

      auto point_laser = particle.pose * data.pose.getOrigin();
      auto hit =
        particle.pose * (data.pose * tf2::Vector3{obs_range * tf2Cos(a), obs_range * tf2Sin(a), 0});

      double pz = 0.0;

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      double z = map_->closest_obstacle(hit);
      assert(!std::isinf(z));

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += config_.z_hit * exp(-(z * z) / (2 * config_.sigma_hit * config_.sigma_hit));
      // Part 2: random measurements
      pz += config_.z_rand / data.range_max;

      // TODO(?): outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;

      if (first) {
        // draw lines from the robot to the ray traced "hit"
        geometry_msgs::Point p1, p2;
        tf2::toMsg(point_laser, p1);
        tf2::toMsg(hit, p2);
        marker.points.push_back(std::move(p1));
        marker.points.push_back(std::move(p2));
        std_msgs::ColorRGBA color;
        color.a = 1;
        color.b = pz;
        color.r = 1 - pz;
        color.r = std::min(z, 1.0);
        marker.colors.push_back(color);
        marker.colors.push_back(std::move(color));
      }
    }

    particle.weight *= p;
    total_weight += particle.weight;
    first = false;
  }

  // Normalize weights
  for (auto & particle : pf->particles) {
    particle.weight /= total_weight;
  }

  debug_pub_.publish(std::move(marker));

  return true;
}