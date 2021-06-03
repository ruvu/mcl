#pragma once

#include <vector>

#include "tf2/LinearMath/Transform.h"

class Particle
{
public:
  tf2::Transform pose;

  Particle() : pose(tf2::Transform::getIdentity()) {}
};

using ParticleFilter = std::vector<Particle>;
