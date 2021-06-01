#include "./differential_motion_model.hpp"

#include "angles/angles.h"
#include "ros/console.h"
#include "tf2/utils.h"

DifferentialMotionModel::DifferentialMotionModel(
  double alpha1, double alpha2, double alpha3, double alpha4)
: sampler_()
{
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
}

void DifferentialMotionModel::odometry_update(
  ParticleFilter * pf, const tf2::Transform pose, const tf2::Transform & delta)
{
  auto [delta_rot1, delta_trans, delta_rot2] = calculate_deltas(delta);
  ROS_DEBUG("delta_rot1: %f", delta_rot1);
  ROS_DEBUG("delta_trans: %f", delta_trans);
  ROS_DEBUG("delta_rot2: %f", delta_rot2);

  for (Particle & particle : *pf) {
    sampler_.setStdDev(alpha1_ * delta_rot1 + alpha2_ * delta_trans);
    double delta_rot1_hat = delta_rot1 - sampler_.sample();
    sampler_.setStdDev(alpha3_ * delta_trans + alpha4_ * delta_rot2);
    double delta_trans_hat = delta_trans - sampler_.sample();
    sampler_.setStdDev(alpha1_ * delta_rot2 + alpha2_ * delta_trans);
    double delta_rot2_hat = delta_rot2 - sampler_.sample();

    tf2::Quaternion q;
    q.setRPY(0, 0, delta_rot1_hat + delta_rot2_hat);
    tf2::Transform new_delta{q, tf2::Vector3{delta_trans_hat, 0, 0}};

    particle.pose *= new_delta;
  }
}

std::array<double, 3> DifferentialMotionModel::calculate_deltas(const tf2::Transform & delta)
{
  // Avoid computing a bearing from two poses that are extremely near each
  // other (happens on in-place rotation).

  double delta_trans = delta.getOrigin().length();

  double delta_rot1;
  if (delta_trans < 0.01) {
    delta_rot1 = 0.0;
  } else {
    delta_rot1 = atan2(delta.getOrigin().getY(), delta.getOrigin().getX());
  }

  double delta_rot2 = tf2::getYaw(delta.getRotation()) - delta_rot1;
  return {delta_rot1, delta_trans, delta_rot2};
}
