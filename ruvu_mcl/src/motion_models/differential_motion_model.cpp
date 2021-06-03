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

    ROS_DEBUG("delta_rot1_hat: %f", delta_rot1_hat);
    ROS_DEBUG("delta_trans_hat: %f", delta_trans_hat);
    ROS_DEBUG("delta_rot2_hat: %f", delta_rot2_hat);

    // first rotate with delta_rot1_hat
    tf2::Quaternion q;
    q.setRPY(0, 0, delta_rot1_hat);
    tf2::Transform new_delta{q};
    // then translate with delta_trans_hat
    new_delta *=
      tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{delta_trans_hat, 0, 0}};
    // finally rotate with  translate with delta_rot2_hat
    q.setRPY(0, 0, delta_rot2_hat);
    new_delta *= tf2::Transform{q};

    ROS_DEBUG(
      "old pose: %f %f", particle.pose.getOrigin().getX(), particle.pose.getOrigin().getY());
    particle.pose *= new_delta;
    ROS_DEBUG(
      "new pose: %f %f", particle.pose.getOrigin().getX(), particle.pose.getOrigin().getY());
  }
}

std::array<double, 3> DifferentialMotionModel::calculate_deltas(const tf2::Transform & delta)
{
  double delta_trans = delta.getOrigin().length();

  // Avoid computing a bearing from two poses that are extremely near each
  // other (happens on in-place rotation).
  double delta_rot1;
  if (delta_trans < 0.01) {
    delta_rot1 = 0.0;
  } else {
    delta_rot1 = atan2(delta.getOrigin().getY(), delta.getOrigin().getX());
  }

  double delta_rot2 = tf2::getYaw(delta.getRotation()) - delta_rot1;
  return {delta_rot1, delta_trans, delta_rot2};
}
