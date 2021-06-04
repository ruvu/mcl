#include "./beam_model.hpp"

#include "../map.hpp"
#include "ros/console.h"

BeamModel::BeamModel(
  double z_hit, double z_short, double z_max, double z_rand, double sigma_hit, double lambda_short,
  double chi_outlier, size_t max_beams, const std::shared_ptr<const Map> & map)
: map_(map)
{
  // TODO: implement
}

bool BeamModel::sensor_update(ParticleFilter * pf, const LaserData & data)
{
  ROS_INFO("BeamModel::sensor_update");
  return true;
}
