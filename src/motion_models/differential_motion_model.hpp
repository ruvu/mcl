#include <array>

#include "motion_model.hpp"

class DifferentialMotionModel : public MotionModel
{
public:
  /**
   * @brief A diff drive constructor
   * @param alpha1 error parameters, see documentation
   * @param alpha2 error parameters, see documentation
   * @param alpha3 error parameters, see documentation
   * @param alpha4 error parameters, see documentation
   */
  DifferentialMotionModel(double alpha1, double alpha2, double alpha3, double alpha4);

  /**
   * @brief Update on new odometry data
   * @param pf The particle filter to update
   * @param pose pose of robot in odometry update
   * @param delta change in pose in odometry update
   */
  void odometry_update(
    ParticleFilter * pf, const tf2::Transform pose, const tf2::Transform & delta) override;

  static std::array<double, 3> calculate_deltas(const tf2::Transform & delta);

private:
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
};
