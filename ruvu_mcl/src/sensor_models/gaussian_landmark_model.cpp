// Copyright 2021 RUVU Robotics B.V.

#include "./gaussian_landmark_model.hpp"

#include <utility>

#include "../particle_filter.hpp"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

/**
 * @brief computes the probability of its argument a under a zero-centered distribution with variance var
 * @param a
 * @param var
 * @return probability
 */
double prob(double a, double var) { return exp(-a * a / 2 / var); }

GaussianLandmarkModel::GaussianLandmarkModel(
  const GaussianLandmarkModelConfig & config, const LandmarkList & landmarks)
: config_(config), landmarks_(landmarks)
{
  assert(config.z_rand >= 0 && config.z_rand <= 1);
  ros::NodeHandle nh("~");
  debug_pub_ = nh.advertise<visualization_msgs::Marker>("gaussian_landmark_model", 1);
}

void GaussianLandmarkModel::sensor_update(ParticleFilter * pf, const LandmarkList & data)
{
  // This algorithm is based on the landmark model known correspondence (Page 150 Probabilistc Robotics)
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
    auto laser_pose = particle.pose * data.pose;

    double p = 1;
    for (const auto & measurement : data.landmarks) {
      auto hit = laser_pose * measurement.pose;

      double pz = 0;
      for (const auto & landmark : landmarks_.landmarks) {
        // Check if ids of landmark and detection agree
        if (landmark.id != measurement.id) continue;

        // Check if incidence angle is within bounds
        auto vector_reflector_to_sensor =
          (laser_pose.getOrigin() - landmark.pose.getOrigin()).normalized();
        auto reflector_direction = landmark.pose.getBasis().getColumn(0);
        if (vector_reflector_to_sensor.dot(reflector_direction) <= 0) continue;

        // range difference
        auto r_hat_diff = (landmark.pose.getOrigin() - hit.getOrigin()).length();

        // bearring difference
        auto t_hat_diff = landmark.pose.getOrigin().angle(hit.getOrigin());

        // likelihood of a landmark measurement
        auto q = prob(r_hat_diff, pow(config_.landmark_sigma_r, 2)) *
                 prob(t_hat_diff, pow(config_.landmark_sigma_t, 2));

        // pick the landmark with the highest probability
        if (q > pz) pz = q;
      }

      pz = (1 - config_.z_rand) * pz + config_.z_rand;

      assert(pz <= 1.0);
      assert(pz >= 0.0);

      p *= pz;

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

    particle.weight *= p;
    total_weight += particle.weight;
    first = false;
  }

  // Normalize weights
  assert(total_weight > 0);
  for (auto & particle : pf->particles) {
    particle.weight /= total_weight;
  }

  debug_pub_.publish(marker);
}
