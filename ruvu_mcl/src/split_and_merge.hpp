// Copyright 2021 RUVU Robotics B.V.
#include "./config.hpp"
#include "./particle_filter.hpp"
#include "map"
#include "math.h"
#include "ros/console.h"
#include "tuple"
#include "vector"

#pragma once
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
