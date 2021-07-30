// Copyright 2021 RUVU Robotics B.V.

#include "./split_and_merge.hpp"

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../particle_filter.hpp"
#include "./utils.hpp"
#include "ros/console.h"
#include "tf2/utils.h"

constexpr auto name = "split_and_merge";

SplitAndMerge::SplitAndMerge(const Config & config)
{
  ROS_INFO_NAMED(name, "Using SplitAndMerge adaptive method");
  adaptive_config_ = std::get<SplitAndMergeConfig>(config.adaptive);
  config_ = config;
}

void SplitAndMerge::merge_particles(ParticleFilter * pf) const
{
  // Based on paper: Monte Carlo localization for mobile robot using adaptive particle merging and splitting technique

  // Discretize particles:
  std::unordered_multimap<Key, Particle, KeyHasher> grid_clusters;
  for (auto & particle : pf->particles) {
    int i = particle.pose.getOrigin().getX() / adaptive_config_.xy_grid_size;
    int j = particle.pose.getOrigin().getY() / adaptive_config_.xy_grid_size;
    int k = tf2::getYaw(particle.pose.getRotation()) /
            adaptive_config_.theta_grid_size;  // TODO(Paul): Combine -pi and +pi clusters
    auto key = std::make_tuple(i, j, k);
    grid_clusters.insert({key, particle});
  }

  // Construct new vector with merged particles:
  std::vector<Particle> merged_particles;
  size_t nr_particles = pf->particles.size();

  // iterate through all elements in the multimap
  // don't worry, a lot of elements are skipped
  for (auto it = grid_clusters.begin(); it != grid_clusters.end();) {
    // get the range of the current key
    const auto range = grid_clusters.equal_range(it->first);
    const auto cluster_size = std::distance(range.first, range.second);
    if (cluster_size == 1 || nr_particles < config_.min_particles + cluster_size - 1) {
      // copy all particles from this cluster
      for (auto it = range.first; it != range.second; ++it) merged_particles.push_back(it->second);
    } else {
      // merge all particles from this cluster
      double sum_x = 0;
      double sum_y = 0;
      double sum_cos_theta = 0;
      double sum_sin_theta = 0;
      double sum_weight = 0;
      for (auto it = range.first; it != range.second; ++it) {
        const auto & particle = it->second;
        sum_x += particle.pose.getOrigin().getX();
        sum_y += particle.pose.getOrigin().getY();
        sum_cos_theta += cos(tf2::getYaw(particle.pose.getRotation()));
        sum_sin_theta += sin(tf2::getYaw(particle.pose.getRotation()));
        sum_weight += particle.weight;
      }
      tf2::Vector3 mean_origin;
      tf2::Quaternion mean_rotation;
      mean_origin.setX(sum_x / cluster_size);
      mean_origin.setY(sum_y / cluster_size);
      double mean_theta = atan2(sum_sin_theta, sum_cos_theta);
      mean_rotation.setRPY(0, 0, mean_theta);
      Particle mean_particle(tf2::Transform(mean_rotation, mean_origin), sum_weight);
      merged_particles.push_back(std::move(mean_particle));
      nr_particles -= cluster_size - 1;
    }

    // step to the end of the range
    it = range.second;
  }
  ROS_DEBUG_NAMED(
    name, "merging reduced particles from %zu to %zu particles", pf->particles.size(),
    merged_particles.size());
  pf->particles = merged_particles;
}

void SplitAndMerge::split_particles(ParticleFilter * pf) const
{
  std::vector<Particle> spawn_particles;
  for (auto & particle : pf->particles) {
    int spawn = particle.weight / adaptive_config_.split_weight;
    spawn = std::min(
      spawn,
      static_cast<int>(config_.max_particles - (pf->particles.size() + spawn_particles.size())));
    if (spawn > 0) {
      particle.weight /= spawn + 1;
      for (int i = 0; i < spawn; i++) {
        spawn_particles.push_back(particle);
      }
    }
  }

  ROS_DEBUG_NAMED(
    name, "splitting increased particles from %zu to %zu particles", pf->particles.size(),
    pf->particles.size() + spawn_particles.size());
  for (const auto & particle : spawn_particles) {
    pf->particles.push_back(particle);
  }
}

int SplitAndMerge::calc_needed_particles(const ParticleFilter & pf) const
{
  return pf.particles.size();
}
