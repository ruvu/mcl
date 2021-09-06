// Copyright 2021 RUVU Robotics B.V.

#include "./landmark_likelihood_field_model.hpp"

#include <math.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include "../map.hpp"
#include "../particle_filter.hpp"
#include "ros/node_handle.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

LandmarkLikelihoodFieldModel::LandmarkLikelihoodFieldModel(
  const LandmarkLikelihoodFieldModelConfig & config, const LandmarkList & landmarks)
: config_(config), landmarks_(landmarks)
{
  assert(config_.z_hit + config_.z_rand <= 1.0);
  ros::NodeHandle nh("~");
  debug_pub_ = nh.advertise<visualization_msgs::Marker>("landmark_likelihood_field_Model", 1);
}

double LandmarkLikelihoodFieldModel::sensor_update(ParticleFilter * pf, const LandmarkList & data)
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

  for (auto & particle : pf->particles) {
    double p = 1.0;
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
        tf2::Transform reflector_orientation(landmark.pose.getRotation());
        auto vector_reflector_orientation = reflector_orientation * tf2::Vector3{1, 0, 0};
        auto incidence_angle = acos(vector_reflector_to_sensor.dot(vector_reflector_orientation));
        if (fabs(incidence_angle) > M_PI / 2) continue;

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
