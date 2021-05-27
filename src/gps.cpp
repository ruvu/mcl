#include <gnuplot-iostream.h>
#include <pf/rv_samp.h>

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

int main()
{
  pf::rvsamp::UnivNormSampler<double> distribution{0, 0.1};
  Model m;
  SensorModel s;

  std::vector<Particle> particles;
  for (int i = 0; i < 10; ++i) {
    particles.emplace_back(distribution.sample(), distribution.sample());
  }

  std::vector<double> model_history;

  const double dt = 0.1;
  for (double t = 0; t < 3; t += dt) {
    m.update(dt);

    for (auto & particle : particles) {
      particle.update(dt);
    }

    double measurement = distribution.sample() * m.height;
    s.update(particles, measurement);

    std::vector<std::vector<Particle>> particle_history;
    std::vector<Particle> particle_positions;
    for (auto & particle : particles) {
      particle_positions.emplace_back(particle);
    }

    model_history.emplace_back(m.height);
    particle_history.emplace_back(std::move(particle_positions));
  }

  Gnuplot gp;
  gp << "plot" << gp.file1d(model_history, "data.txt") << "with lines title 'ground truth'\n";
  // gp << "plot" << gp.file1d(particle_history, "particles.txt") << "title 'particles'\n";
}
