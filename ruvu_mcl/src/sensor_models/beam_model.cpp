#include "./beam_model.hpp"

#include "../map.hpp"
#include "ros/console.h"

BeamModel::BeamModel(const Parameters & parameters, const std::shared_ptr<const Map> & map)
: parameters_(parameters), map_(map)
{
  // TODO: implement
}

bool BeamModel::sensor_update(ParticleFilter * pf, const LaserData & data)
{
  ROS_INFO("BeamModel::sensor_update");
  auto step = (data.ranges.size() - 1) / (parameters_.max_beams - 1);

  ROS_INFO("step size=%zu", step);
  for (auto & particle : *pf) {
    for (std::size_t i = 0; i < data.ranges.size(); i += step) {
      auto range = data.get_range(i);
      auto obs_range = data.ranges[i];
      range = data.pose * range;      // transform to base_link
      range = particle.pose * range;  // transform to map

      auto map_range = map_->calc_range(data.pose.getOrigin(), range);

      ROS_INFO("obs: %f map: %f", obs_range, map_range);
    }
  }

  return true;
}
