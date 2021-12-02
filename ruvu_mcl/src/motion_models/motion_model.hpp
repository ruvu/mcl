// Copyright 2021 RUVU Robotics B.V.

#pragma once

// forward declare
namespace tf2
{
class Transform;
}

namespace ruvu_mcl
{
class ParticleFilter;

/**
 * @brief Base class for all motion models
 *
 * Base class for all motion models. A motion model tries to accurately model the specific types of
 * uncertainty that exist in robot actuation.
 *
 * Motion models take odometry measurements as input and move the particles according to a specific
 * probabalistic distribution. See also chapter 5 of Probabilistc Robotics.
 */
class MotionModel
{
public:
  virtual ~MotionModel();

  /**
   * @brief Update particles based on new odometry data
   * @param pf The particle filter to update
   * @param delta change in pose in odometry update
   */
  virtual void odometry_update(ParticleFilter * pf, const tf2::Transform & delta) = 0;
};
}  // namespace ruvu_mcl
