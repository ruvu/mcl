// Copyright 2021 RUVU Robotics B.V.

#include "./split_and_merge.hpp"

#include <algorithm>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "../particle_filter.hpp"
#include "ros/console.h"
#include "tf2/utils.h"

constexpr auto name = "split_and_merge";

SplitAndMerge::SplitAndMerge(const Config & config)
{
  ROS_INFO_NAMED(name, "Using SplitAndMerge adaptive method");
  adaptive_config_ = std::get<SplitAndMergeConfig>(config.adaptive);
  config_ = config;
}

void SplitAndMerge::merge_particles(ParticleFilter * pf)
{
  // Based on paper: Monte Carlo localization for mobile robot using adaptive particle merging and splitting technique

  // Discretize particles:
  std::map<std::tuple<int, int, int>, std::vector<Particle> > grid_clusters;
  for (auto & particle : pf->particles) {
    int i = particle.pose.getOrigin().getX() / adaptive_config_.xy_grid_size;
    int j = particle.pose.getOrigin().getY() / adaptive_config_.xy_grid_size;
    int k = tf2::getYaw(particle.pose.getRotation()) /
            adaptive_config_.theta_grid_size;  // TODO(Paul): Combine -pi and +pi clusters
    auto key = std::make_tuple(i, j, k);
    grid_clusters[key].push_back(std::move(particle));
  }

  // Construct new vector with merged particles:
  std::vector<Particle> merged_particles;
  size_t nr_particles = pf->particles.size();
  for (const auto & cluster : grid_clusters) {
    if (
      cluster.second.size() == 1 ||
      nr_particles < config_.min_particles + cluster.second.size() - 1) {
      merged_particles.insert(merged_particles.end(), cluster.second.begin(), cluster.second.end());
    } else {
      double sum_x = 0;
      double sum_y = 0;
      double sum_cos_theta = 0;
      double sum_sin_theta = 0;
      double sum_weight = 0;
      for (auto & particle : cluster.second) {
        sum_x += particle.pose.getOrigin().getX();
        sum_y += particle.pose.getOrigin().getY();
        sum_cos_theta += cos(tf2::getYaw(particle.pose.getRotation()));
        sum_sin_theta += sin(tf2::getYaw(particle.pose.getRotation()));
        sum_weight += particle.weight;
      }
      tf2::Vector3 mean_origin;
      tf2::Quaternion mean_rotation;
      mean_origin.setX(sum_x / cluster.second.size());
      mean_origin.setY(sum_y / cluster.second.size());
      double mean_theta = atan2(sum_sin_theta, sum_cos_theta);
      mean_rotation.setRPY(0, 0, mean_theta);
      Particle mean_particle(tf2::Transform(mean_rotation, mean_origin), sum_weight);
      merged_particles.push_back(std::move(mean_particle));
      nr_particles -= cluster.second.size() - 1;
      ROS_DEBUG_NAMED(name, "Merged %lu particles", cluster.second.size());
    }
  }
  pf->particles = merged_particles;
}

void SplitAndMerge::split_particles(ParticleFilter * pf)
{
  std::vector<Particle> spawn_particles;
  for (auto & particle : pf->particles) {
    size_t spawn = particle.weight / adaptive_config_.split_weight;
    spawn =
      std::min(spawn, (config_.max_particles - (pf->particles.size() + spawn_particles.size())));
    if (spawn) {
      ROS_DEBUG_NAMED(name, "Split particle into %lu equals", spawn + 1);
      particle.weight /= spawn + 1;
      for (size_t i = 0; i < spawn; i++) {
        spawn_particles.push_back(std::move(particle));
      }
    }
  }

  for (auto particle : spawn_particles) {
    pf->particles.push_back(std::move(particle));
  }
  ROS_DEBUG_NAMED(name, "Using %lu particles", pf->particles.size());
}
