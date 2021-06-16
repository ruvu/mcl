// Copyright 2021 RUVU Robotics B.V.

#include <gnuplot-iostream.h>

#include <memory>
#include <utility>
#include <vector>

#include "../src/particle_filter.hpp"
#include "../src/resamplers/low_variance.hpp"
#include "../src/rng.hpp"

auto convert_to_gnuplot(const ParticleFilter & pf)
{
  std::vector<std::pair<double, double>> points;
  for (const auto & particle : pf.particles) {
    const auto & o = particle.pose.getOrigin();
    points.emplace_back(o.getX(), o.getY());
  }
  return points;
}

int main()
{
  Gnuplot gp;
  gp << "set xrange [-2:2]\n";
  gp << "set yrange [-2:2]\n";
  gp << "set multiplot layout 2,2 rowsfirst\n";

  auto rng = std::make_shared<Rng>();
  auto initial_dist = [rng]() { return rng->sample_uniform_distribution(-1, 1); };

  ParticleFilter pf;
  for (int i = 0; i < 1000; ++i) {
    double x = initial_dist();
    double y = initial_dist();
    double weight = 1 / hypot(x, y);
    pf.particles.emplace_back(
      tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{x, y, 0}}, double{weight});
  }
  pf.normalize_weights();

  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'before'\n";

  // Low-variance resampling (Page 86 Probabilistc Robotics)
  LowVariance resampler{rng};
  for (int j = 0; j < 3; j++) {
    resampler.resample(&pf);
    gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'Resampling #: " << j + 1 << "'\n";
  }
}
