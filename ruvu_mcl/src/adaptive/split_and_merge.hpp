// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include "../config.hpp"
#include "./adaptive_method.hpp"

namespace ruvu_mcl
{
// forward declare
class ParticleFilter;

/**
 * @brief Use split & merge to adapt the number of particles
 *
 * From the paper: Monte Carlo localization for mobile robot using adaptive particle merging and
 * splitting technique
 */
class SplitAndMerge : public AdaptiveMethod
{
public:
  explicit SplitAndMerge(const Config & config);

  void after_odometry_update(ParticleFilter * pf) const override { merge_particles(pf); }
  void after_sensor_update(ParticleFilter * pf) const override { split_particles(pf); }
  int calc_needed_particles(const ParticleFilter & pf) const override;

  /**
   * @brief Merge particles that are close together
   */
  void merge_particles(ParticleFilter * pf) const;

  /**
   * @brief Split particles that have a high weight
   */
  void split_particles(ParticleFilter * pf) const;

private:
  SplitAndMergeConfig adaptive_config_;
  Config config_;
};
}  // namespace ruvu_mcl
