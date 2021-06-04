#pragma once

#include <memory>

#include "./laser.hpp"

// forward declare
class Map;

class BeamModel : public Laser
{
public:
  /*
   * @brief BeamModel constructor
   */
  BeamModel(
    double z_hit, double z_short, double z_max, double z_rand, double sigma_hit,
    double lambda_short, double chi_outlier, size_t max_beams,
    const std::shared_ptr<const Map> & map);

  /*
   * @brief Run a sensor update on laser
   * @param pf Particle filter to use
   * @param data Laser data to use
   * @return if it was succesful
   */
  bool sensor_update(ParticleFilter * pf, const LaserData & data) override;

private:
  const std::shared_ptr<const Map> map_;
};
