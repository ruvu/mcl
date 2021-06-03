#pragma once

#include <tf2/LinearMath/Transform.h>

#include <vector>

class Particle
{
public:
  tf2::Transform pose;

  Particle() : pose(tf2::Transform::getIdentity()) {}
};

using ParticleFilter = std::vector<Particle>;
