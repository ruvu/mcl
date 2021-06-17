// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <vector>

#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"

class Particle
{
public:
  tf2::Transform pose;
  double_t weight;

  Particle(const tf2::Transform & pose, double weight) : pose(pose), weight(weight) {}
};

class ParticleFilter
{
public:
  std::vector<Particle> particles;

  void normalize_weights();
  double calc_effective_sample_size();
  std::array<double, 36> get_2d_covariance_array();
};
