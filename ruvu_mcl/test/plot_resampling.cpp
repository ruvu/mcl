#include <gnuplot-iostream.h>

#include "../src/particle_filter.hpp"
#include "ros/console.h"
#include <math.h>
#include <random>

auto convert_to_gnuplot(const ParticleFilter & pf)
{
  std::vector<std::pair<double, double>> points;
  for (const auto & particle : pf) {
    const auto & o = particle.pose.getOrigin();
    points.emplace_back(o.getX(), o.getY());
  }
  return points;
}

auto normalize_weights(ParticleFilter & pf){
    double total_weight = 0;
    for (const auto & particle : pf) {
        total_weight += particle.weight;
    }
    for (auto & particle : pf) {
        particle.weight /= total_weight;
    }
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

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> initial_dist(-1, 1);

  ParticleFilter pf;
  for (int i = 0; i < 1000; ++i) {
    double x = initial_dist(gen);
    double y = initial_dist(gen);
    double weight = 1 / hypot(x, y);
    pf.emplace_back(tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{x, y, 0}}, double{weight});
  }
  normalize_weights(pf);

  gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'before'\n";

  // Low-variance resampling (Page 86 Probabilistc Robotics)
  double M = pf.size();
  double M_inv = 1. / M;
  std::uniform_real_distribution<> uniform_dist(0, M_inv);
  for (int j=0; j<3; j++){
    ParticleFilter pf_resampled;
    float r = uniform_dist(gen);
    double c = pf.at(0).weight;
    int i = 0;
    for (int m = 0; m < M; m++){
        float u = r + m * M_inv;
        while (u > c){
            i++;
            c += pf.at(i).weight;
        }
        pf_resampled.emplace_back(pf.at(i));
    }
    normalize_weights(pf_resampled);
    pf = pf_resampled;
    gp << "plot" << gp.file1d(convert_to_gnuplot(pf)) << "title 'Resampling #: " << j+1 <<"'\n";

  }

}