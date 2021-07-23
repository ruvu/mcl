// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include "../config.hpp"
#include "./adaptive_method.hpp"

// forward declare
class ParticleFilter;

class SplitAndMerge : public AdaptiveMethod
{
public:
  SplitAndMerge(const Config & config);

  void after_odometry_update(ParticleFilter * pf) override { merge_particles(pf); }
  void after_sensor_update(ParticleFilter * pf) override { split_particles(pf); }

  void merge_particles(ParticleFilter * pf);
  void split_particles(ParticleFilter * pf);

private:
  SplitAndMergeConfig adaptive_config_;
  Config config_;
};
