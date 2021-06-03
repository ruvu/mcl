#include "../particle_filter.hpp"
#include "tf2/LinearMath/Transform.h"

class MotionModel
{
public:
  virtual ~MotionModel() = default;

  /**
   * @brief Update on new odometry data
   * @param pf The particle filter to update
   * @param pose pose of robot in odometry update
   * @param delta change in pose in odometry update
   */
  virtual void odometry_update(
    ParticleFilter * pf, const tf2::Transform pose, const tf2::Transform & delta) = 0;
};
