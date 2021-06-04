#include <gnuplot-iostream.h>

#include "../src/particle_filter.hpp"
#include "ros/console.h"

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
  gp << "set multiplot layout 2,1 rowsfirst\n";

  ParticleFilter pf;
  for (int i = 0; i < 100; ++i) {
    pf.emplace_back(tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{1, 0, 0}});
  }

  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'before'\n";

  // TODO resample

  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'after'\n";
}
