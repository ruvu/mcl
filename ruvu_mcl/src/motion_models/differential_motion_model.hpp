// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <array>
#include <memory>

#include "../config.hpp"
#include "./motion_model.hpp"

namespace ruvu_mcl
{
// forward declare
class Rng;

/**
 * @brief Motion model for a differential drive
 */
class DifferentialMotionModel : public MotionModel
{
public:
  /**
   * @brief Construct a differental drive motion model
   * @param alpha1 error parameters, see documentation
   * @param alpha2 error parameters, see documentation
   * @param alpha3 error parameters, see documentation
   * @param alpha4 error parameters, see documentation
   * @param rng random number generator
   */
  DifferentialMotionModel(
    double alpha1, double alpha2, double alpha3, double alpha4, const std::shared_ptr<Rng> & rng);

  /**
   * @brief Construct a differental drive motion model
   * @param config parameters
   * @param rng random number generator
   */
  DifferentialMotionModel(
    const DifferentialMotionModelConfig & config, const std::shared_ptr<Rng> & rng);

  void odometry_update(ParticleFilter * pf, const tf2::Transform & delta) override;

  /**
   * @brief Compute {delta_rot1, delta_trans, delta_rot2} from a displacement
   * @param delta displacement
   * @return {delta_rot1, delta_trans, delta_rot2}
   */
  static std::array<double, 3> calculate_deltas(const tf2::Transform & delta);

private:
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;

  std::shared_ptr<Rng> rng_;
};
}  // namespace ruvu_mcl
