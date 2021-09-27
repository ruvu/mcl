// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include "../config.hpp"
#include "./adaptive_method.hpp"

// forward declare
class ParticleFilter;

class SplitAndMerge : public AdaptiveMethod
{
public:
  explicit SplitAndMerge(const Config & config);

  void after_odometry_update(ParticleFilter * pf) const override { merge_particles(pf); }
  void after_sensor_update(ParticleFilter * pf) const override { split_particles(pf); }
  int calc_needed_particles(const ParticleFilter & pf) const override;

  void merge_particles(ParticleFilter * pf) const;
  void split_particles(ParticleFilter * pf) const;

private:
  SplitAndMergeConfig adaptive_config_;
  Config config_;
};
