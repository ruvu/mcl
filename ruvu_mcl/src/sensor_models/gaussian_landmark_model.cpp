// Copyright 2021 RUVU Robotics B.V.

#include "./gaussian_landmark_model.hpp"

#include <boost/math/special_functions/erf.hpp>
#include <limits>
#include <utility>
#include <vector>

#include "../config.hpp"
#include "../particle_filter.hpp"
#include "ros/node_handle.h"
#include "ruvu_mcl_msgs/ParticleStatistics.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

constexpr auto name = "gaussian_landmark_model";

namespace ruvu_mcl
{
/**
 * @brief computes the probability of its argument a under a zero-centered distribution with variance var
 * @param a
 * @param var
 * @return probability
 */
double prob(double a, double var) { return exp(-a * a / 2 / var); }

/**
 * @brief Quantile function
 *
 * A normal random variable X will exceed mu + F_inv(p) * sigma with probability 1 - p.
 * @param p probability
 */
double F_inv(double p) { return sqrt(2) * boost::math::erf_inv(2 * p - 1); }

KDTreeType landmarks_to_kdtree(const std::vector<Landmark> & landmarks)
{
  KDTreeType tree;
  for (const auto & landmark : landmarks) {
    tree.insert(KDTreeNode{landmark});
  }
  return tree;
}

GaussianLandmarkModel::GaussianLandmarkModel(
  const GaussianLandmarkModelConfig & config, const LandmarkList & map)
: z_rand_(config.z_rand),
  landmark_var_r_(config.landmark_sigma_r * config.landmark_sigma_r),
  landmark_var_t_(config.landmark_sigma_t * config.landmark_sigma_t),
  global_frame_id_(config.global_frame_id),
  tree_(landmarks_to_kdtree(map.landmarks))
{
  assert(config.z_rand >= 0 && config.z_rand <= 1);

  max_confidence_range_ = config.landmark_sigma_r * F_inv(config.landmark_max_r_confidence);
  ROS_INFO_NAMED(
    name, "%f confidence is %f meter (%f sigma)", config.landmark_max_r_confidence,
    max_confidence_range_, max_confidence_range_ / config.landmark_sigma_r);

  if (ros::isInitialized()) {
    ros::NodeHandle nh("~");
    debug_pub_ = nh.advertise<visualization_msgs::Marker>("gaussian_landmark_model", 1);
    statistics_pub_ = nh.advertise<ruvu_mcl_msgs::ParticleStatistics>("sensor_model_statistics", 1);
  }
}

void GaussianLandmarkModel::sensor_update(ParticleFilter * pf, const LandmarkList & data)
{
  // This algorithm is based on the landmark model known correspondence (Page 150 Probabilistc Robotics)
  if (data.landmarks.empty()) return;

  visualization_msgs::Marker marker;
  if (debug_pub_.getNumSubscribers()) {
    marker.header.frame_id = global_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::MODIFY;
    tf2::toMsg(tf2::Transform::getIdentity(), marker.pose);
    marker.scale.x = 0.01;
  }

  bool first = true;  // publish debug info for the first particle
  double total_weight = 0.0;
  ruvu_mcl_msgs::ParticleStatistics statistics;

  for (auto & particle : pf->particles) {
    auto laser_pose = particle.pose * data.pose;

    double p = 1;
    for (const auto & measurement : data.landmarks) {
      double pz_arg = std::numeric_limits<double>::max();
      auto measurement_length = measurement.pose.getOrigin().length();
      // Only landmarks withing max_range around the laser have a chance to influence the particle
      // weight. We can use this to quickly prune the map with a KDTree.
      double max_range = measurement.pose.getOrigin().length() + max_confidence_range_;
      const auto & l = laser_pose.getOrigin();
      std::vector<KDTreeNode> results;
      KDTreeNode node{Landmark{tf2::Transform{
        tf2::Quaternion::getIdentity(),
        tf2::Vector3{static_cast<float>(l.x()), static_cast<float>(l.y()), 0}}}};
      tree_.find_within_range(node, max_range, std::back_inserter(results));
      for (const auto & result : results) {
        auto & landmark = result.landmark;

        // Check if ids of landmark and detection agree
        if (landmark.id != measurement.id) continue;

        // Check if incidence angle is within bounds
        auto vector_reflector_to_sensor =
          (laser_pose.getOrigin() - landmark.pose.getOrigin()).normalized();
        auto reflector_direction = landmark.pose.getBasis().getColumn(0);
        if (vector_reflector_to_sensor.dot(reflector_direction) <= 0) continue;

        auto landmark_pose_LASER = laser_pose.inverseTimes(landmark.pose);

        // range difference
        auto r_hat_diff = landmark_pose_LASER.getOrigin().length() - measurement_length;

        // bearring difference
        auto t_hat_diff = landmark_pose_LASER.getOrigin().angle(measurement.pose.getOrigin());

        // Compare argument of likelihood of a landmark measurement
        auto q = r_hat_diff * r_hat_diff / 2 / landmark_var_r_ +
                 t_hat_diff * t_hat_diff / 2 / landmark_var_t_;

        // pick the landmark with the lowest probability argument
        if (q < pz_arg) pz_arg = q;
      }

      // Calculate exp() once for computational efficiency
      double pz = exp(-pz_arg);
      pz = (1 - z_rand_) * pz + z_rand_;

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      p *= pz;

      if (first) {
        // draw lines from the robot to the ray traced "hit"
        geometry_msgs::Point p1, p2;
        tf2::toMsg(particle.pose.getOrigin(), p1);
        tf2::toMsg(laser_pose * measurement.pose.getOrigin(), p2);
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

    // Gather data for sensor model statistics
    if (statistics_pub_.getNumSubscribers()) {
      statistics.weight_updates.push_back(p);
    }

    particle.weight *= p;
    total_weight += particle.weight;
    first = false;
  }

  // Normalize weights
  pf->normalize_weights(total_weight);

  if (debug_pub_.getNumSubscribers()) debug_pub_.publish(marker);
  if (statistics_pub_.getNumSubscribers()) {
    statistics.sensor_model = typeid(this).name();
    statistics_pub_.publish(std::move(statistics));
  }
}
}  // namespace ruvu_mcl
