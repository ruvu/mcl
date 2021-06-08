#pragma once

#include <memory>

#include "./laser.hpp"
#include "ros/publisher.h"

// forward declare
class Map;

class BeamModel : public Laser
{
public:
  struct Parameters
  {
    double z_hit;
    double z_short;
    double z_max;
    double z_rand;
    double sigma_hit;
    double lambda_short;
    double chi_outlier;
    size_t max_beams;
  };

  /*
   * @brief BeamModel constructor
   */
  BeamModel(const Parameters & parameters, const std::shared_ptr<const Map> & map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  double sensor_update(ParticleFilter * pf, const LaserData & data) override;

private:
  const Parameters parameters_;
  const std::shared_ptr<const Map> map_;
  ros::Publisher debug_pub_;
};
