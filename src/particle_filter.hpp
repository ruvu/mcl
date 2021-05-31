#include <tf2/LinearMath/Transform.h>

#include <vector>

class Particle
{
public:
  tf2::Transform pose;
};

using ParticleFilter = std::vector<Particle>;
