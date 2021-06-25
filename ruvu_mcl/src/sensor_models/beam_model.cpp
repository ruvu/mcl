// Copyright 2021 RUVU Robotics B.V.

#include "./beam_model.hpp"

#include <memory>
#include <utility>

#include "../map.hpp"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

BeamModel::BeamModel(
  const BeamModelConfig & config, const std::shared_ptr<const OccupancyMap> & map)
: parameters_(config), map_(map)
{
  assert(parameters_.z_hit + parameters_.z_short + parameters_.z_max + parameters_.z_rand <= 1.0);
  ros::NodeHandle nh("~");
  debug_pub_ = nh.advertise<visualization_msgs::Marker>("beam_model", 1);
}

double BeamModel::sensor_update(ParticleFilter * pf, const LaserData & data)
{
  // inspired by:
  // https://github.com/ros-planning/navigation2/blob/f23d915608a94039ff91008014730971a8795c15/nav2_amcl/src/sensors/laser/beam_model.cpp

  visualization_msgs::Marker marker;
  marker.header.frame_id = parameters_.global_frame_id;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  tf2::toMsg(tf2::Transform::getIdentity(), marker.pose);
  marker.scale.x = 0.01;

  bool first = true;  // publish debug info for the first particle
  double total_weight = 0.0;
  auto step = (data.ranges.size() - 1) / (parameters_.max_beams - 1);
  for (auto & particle : pf->particles) {
    double p = 1.0;

    for (std::size_t i = 0; i < data.ranges.size(); i += step) {
      double obs_range = data.ranges[i];
      auto a = data.get_angle(i);

      if (std::isinf(obs_range)) continue;  // TODO(Ramon): don't skip infs

      auto point_laser = particle.pose * data.pose.getOrigin();
      auto point_hit =
        particle.pose *
        (data.pose * tf2::Vector3{data.range_max * tf2Cos(a), data.range_max * tf2Sin(a), 0});
      double map_range = map_->calc_range(point_laser, point_hit);
      if (std::isinf(map_range)) continue;  // TODO(Ramon): don't skip infs

      double pz = 0.0;

      // Part 1: good, but noisy, hit
      auto z = obs_range - map_range;
      pz += parameters_.z_hit * exp(-(z * z) / (2 * parameters_.sigma_hit * parameters_.sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if (z < 0) {
        pz += parameters_.z_short * parameters_.lambda_short *
              exp(-parameters_.lambda_short * obs_range);
      }

      // Part 3: Failure to detect obstacle, reported as max-range
      if (obs_range == data.range_max) {
        pz += parameters_.z_max * 1.0;
      }

      // Part 4: Random measurements
      if (obs_range < data.range_max) {
        pz += parameters_.z_rand * 1.0 / data.range_max;
      }

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
        tf2::toMsg(point_laser);
        tf2::toMsg(point_hit);
        marker.points.push_back(std::move(p1));
        marker.points.push_back(std::move(p2));
        std_msgs::ColorRGBA color;
        color.a = 1;
        color.b = pz;
        color.r = 1 - pz;
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
