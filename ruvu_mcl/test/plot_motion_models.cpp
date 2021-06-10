// Copyright 2021 RUVU Robotics B.V.

#include <gnuplot-iostream.h>

#include "../src/motion_models/differential_motion_model.hpp"
#include "../src/rng.hpp"
#include "ros/console.h"

ParticleFilter run_differential_motion_model(MotionModel && model)
{
  ParticleFilter pf;
  int n = 100;
  for (int i = 0; i < n; ++i) {
    pf.emplace_back(tf2::Transform::getIdentity(), 1. / n);
  }

  model.odometry_update(
    &pf, tf2::Transform::getIdentity(),
    tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{1, 0, 0}});

  return pf;
}

auto convert_to_gnuplot(const ParticleFilter & pf)
{
  std::vector<std::pair<double, double>> points;
  for (const auto & particle : pf) {
    const auto & o = particle.pose.getOrigin();
    points.emplace_back(o.getX(), o.getY());
  }
  return points;
}

int main()
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  Gnuplot gp;
  gp << "set xrange [-2:2]\n";
  gp << "set yrange [-2:2]\n";
  gp << "set multiplot layout 2,2 rowsfirst\n";
  auto rng = std::make_shared<Rng>();
  auto pf = run_differential_motion_model(DifferentialMotionModel{0.5, 0.1, 0.1, 0.1, rng});
  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'alpha1'\n";
  pf = run_differential_motion_model(DifferentialMotionModel{0.1, 0.5, 0.1, 0.1, rng});
  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'alpha2'\n";
  pf = run_differential_motion_model(DifferentialMotionModel{0.1, 0.1, 0.5, 0.1, rng});
  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'alpha3'\n";
  pf = run_differential_motion_model(DifferentialMotionModel{0.1, 0.1, 0.1, 0.5, rng});
  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'alpha4'\n";
}
