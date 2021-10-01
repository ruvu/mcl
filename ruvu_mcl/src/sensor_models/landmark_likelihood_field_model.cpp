// Copyright 2021 RUVU Robotics B.V.

#include "./landmark_likelihood_field_model.hpp"

#include <limits>

#include "../particle_filter.hpp"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

LandmarkLikelihoodFieldModel::LandmarkLikelihoodFieldModel(
  const LandmarkLikelihoodFieldModelConfig & config, const LandmarkList & landmarks)
: config_(config), landmarks_(landmarks)
{
  assert(config_.z_hit + config_.z_rand <= 1.0);
  ros::NodeHandle nh("~");
  debug_pub_ = nh.advertise<visualization_msgs::Marker>("landmark_likelihood_field_model", 1);
}

void LandmarkLikelihoodFieldModel::sensor_update(ParticleFilter * pf, const LandmarkList & data)
{
  // This algorithm is based on the likelihood field range finder model (Page 143 Probabilistc Robotics)
  if (data.landmarks.empty()) return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = config_.global_frame_id;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  tf2::toMsg(tf2::Transform::getIdentity(), marker.pose);
  marker.scale.x = 0.01;

  bool first = true;  // publish debug info for the first particle
  double total_weight = 0.0;

  for (auto & particle : pf->particles) {
    double p = 0.0;
    auto laser_pose = particle.pose * data.pose;

    for (const auto & measurement : data.landmarks) {
      double pz = 0.0;

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      auto hit = laser_pose * measurement.pose;
      double z = std::numeric_limits<double>::infinity();
      for (const auto & landmark : landmarks_.landmarks) {
        // Check if ids of landmark and detection agree
        if (landmark.id != measurement.id) continue;

        // Check if incidence angle is within bounds
        auto vector_reflector_to_sensor =
          (laser_pose.getOrigin() - landmark.pose.getOrigin()).normalized();
        auto reflector_direction = landmark.pose.getBasis().getColumn(0);
        if (vector_reflector_to_sensor.dot(reflector_direction) <= 0) continue;

        auto distance = (landmark.pose.getOrigin() - hit.getOrigin()).length();
        if (distance < z) z = distance;
      }

      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += config_.z_hit * exp(-(z * z) / (2 * config_.sigma_hit * config_.sigma_hit));
      // Part 2: random measurements
      pz += config_.z_rand;

      // TODO(?): outlier rejection for short readings
      // TODO(?): Use covariance for weighing measurement influence?

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;

      if (first) {
        // draw lines from the robot to the ray traced "hit"
        geometry_msgs::Point p1, p2;
        tf2::toMsg(particle.pose.getOrigin(), p1);
        tf2::toMsg(hit.getOrigin(), p2);
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        std_msgs::ColorRGBA color;
        color.a = 1;
        color.b = pz;
        color.r = 1 - pz;
        marker.colors.push_back(color);
        marker.colors.push_back(color);
      }
    }

    // Normalize weight update
    p /= data.landmarks.size();

    particle.weight *= p;
    total_weight += particle.weight;
    first = false;
  }

  // Normalize weights
  for (auto & particle : pf->particles) {
    particle.weight /= total_weight;
  }

  debug_pub_.publish(marker);
}
