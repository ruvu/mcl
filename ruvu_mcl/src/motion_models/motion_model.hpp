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

class MotionModel
{
public:
  virtual ~MotionModel();

  /**
   * @brief Update on new odometry data
   * @param pf The particle filter to update
   * @param delta change in pose in odometry update
   */
  virtual void odometry_update(ParticleFilter * pf, const tf2::Transform & delta) = 0;
};
}  // namespace ruvu_mcl
