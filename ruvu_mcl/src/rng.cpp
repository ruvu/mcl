#include "./rng.hpp"

Rng::Rng() : generator(std::random_device{}()) {}

double Rng::sample_normal_distribution(double mean, double stddev)
{
  std::normal_distribution<> distribution{mean, stddev};
  return distribution(generator);
}

double Rng::sample_uniform_distribution(double a, double b)
{
  std::uniform_real_distribution<> distribution{a, b};
  return distribution(generator);
}
