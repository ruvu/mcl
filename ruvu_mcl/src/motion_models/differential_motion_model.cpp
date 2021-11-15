// Copyright 2021 RUVU Robotics B.V.

#include "./differential_motion_model.hpp"

#include <memory>

#include "../particle_filter.hpp"
#include "../rng.hpp"
#include "angles/angles.h"
#include "ros/console.h"
#include "tf2/utils.h"

constexpr auto name = "differential_motion_model";

namespace ruvu_mcl
{
DifferentialMotionModel::DifferentialMotionModel(
  double alpha1, double alpha2, double alpha3, double alpha4, const std::shared_ptr<Rng> & rng)
: rng_(rng)
{
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
}

DifferentialMotionModel::DifferentialMotionModel(
  const DifferentialMotionModelConfig & config, const std::shared_ptr<Rng> & rng)
: DifferentialMotionModel(config.alpha1, config.alpha2, config.alpha3, config.alpha4, rng)
{
}

void DifferentialMotionModel::odometry_update(ParticleFilter * pf, const tf2::Transform & delta)
{
  auto [delta_rot1, delta_trans, delta_rot2] = calculate_deltas(delta);

  for (Particle & particle : pf->particles) {
    double delta_rot1_hat =
      rng_->sample_normal_distribution(delta_rot1, alpha1_ * delta_rot1 + alpha2_ * delta_trans);
    double delta_trans_hat =
      rng_->sample_normal_distribution(delta_trans, alpha3_ * delta_trans + alpha4_ * delta_rot2);
    double delta_rot2_hat =
      rng_->sample_normal_distribution(delta_rot2, alpha1_ * delta_rot2 + alpha2_ * delta_trans);

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

    particle.pose *= new_delta;
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

  // backwards driving
  if (delta_rot1 >= M_PI / 2 || delta_rot1 <= -M_PI / 2) {
    delta_rot1 = angles::normalize_angle(delta_rot1 + M_PI);
    delta_trans = -delta_trans;
  }

  double delta_rot2 = tf2::getYaw(delta.getRotation()) - delta_rot1;

  ROS_DEBUG_NAMED(name, "delta_rot1: %f", delta_rot1);
  ROS_DEBUG_NAMED(name, "delta_trans: %f", delta_trans);
  ROS_DEBUG_NAMED(name, "delta_rot2: %f", delta_rot2);
  return {delta_rot1, delta_trans, delta_rot2};
}
}  // namespace ruvu_mcl
