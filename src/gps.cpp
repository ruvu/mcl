#include <pf/rv_samp.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <cstdio>
#include <iostream>

using Input = Eigen::Matrix<double, 2, 1>;

static constexpr double g = 9.81;

class Model
{
public:
  double height = 0;
  double velocity = 0;

  void update(double dt)
  {
    velocity += -g * dt;
    height += velocity * dt;
  }

private:
};

class Particle
{
public:
  double height;
  double velocity;
  double weight = 1;

  explicit Particle(double height, double velocity) : height(height), velocity(velocity) {}
  void update(double dt)
  {
    velocity += -g * dt;
    height += velocity * dt;
  }
};

class SensorModel
{
public:
  void update(std::vector<Particle> & particles, double data)
  {
    for (auto & particle : particles) {
      auto z = particle.height - data;
      auto p = exp(-z * z / 2 / sigma / sigma);
      particle.weight *= p;
    }
  }

private:
  const double sigma = 1;
};

class Gps
{
public:
  Gps(const ros::NodeHandle & nh, const ros::NodeHandle & private_nh)
  {
    for (int i = 0; i < 10; ++i) {
      particles_.emplace_back(distribution.sample(), distribution.sample());
    }
  }

private:
  void update(const ros::Time & dt)
  {
    model_.update(0.1);

    for (auto & particle : particles_) {
      particle.update(0.1);
    }

    double measurement = distribution.sample() * model_.height;
    sensor_model_.update(particles_, measurement);
  }

  pf::rvsamp::UnivNormSampler<double> distribution{0, 0.1};
  std::vector<Particle> particles_;
  Model model_ = {};
  SensorModel sensor_model_ = {};
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gps");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  Gps(nh, private_nh);
  ros::spin();
  return EXIT_SUCCESS;
}
