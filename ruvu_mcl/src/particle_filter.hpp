#pragma once

#include <vector>

#include "tf2/LinearMath/Transform.h"

class Particle
{
public:
  tf2::Transform pose;
  double_t weight;

  Particle(const tf2::Transform & pose, double weight) : pose(pose), weight(weight) {}
};

using ParticleFilter = std::vector<Particle>;
