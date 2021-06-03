#include <gnuplot-iostream.h>

#include "../src/motion_models/differential_motion_model.hpp"
#include "ros/console.h"

ParticleFilter run_differential_motion_model(MotionModel && model)
{
  ParticleFilter pf;
  for (int i = 0; i < 100; ++i) {
    pf.emplace_back();
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
  auto pf = run_differential_motion_model(DifferentialMotionModel{0.5, 0.1, 0.1, 0.1});
  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'alpha1'\n";
  pf = run_differential_motion_model(DifferentialMotionModel{0.1, 0.5, 0.1, 0.1});
  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'alpha2'\n";
  pf = run_differential_motion_model(DifferentialMotionModel{0.1, 0.1, 0.5, 0.1});
  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'alpha3'\n";
  pf = run_differential_motion_model(DifferentialMotionModel{0.1, 0.1, 0.1, 0.5});
  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'alpha4'\n";
}
