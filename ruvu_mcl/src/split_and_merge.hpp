// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include "./config.hpp"

// forward declare
class ParticleFilter;

class SplitAndMerge
{
public:
  SplitAndMerge(const Config & config);
  void merge_particles(ParticleFilter * pf);
  void split_particles(ParticleFilter * pf);

private:
  SplitAndMergeConfig adaptive_config_;
  Config config_;
};
