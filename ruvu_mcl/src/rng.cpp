// Copyright 2021 RUVU Robotics B.V.

#include "./rng.hpp"

namespace ruvu_mcl
{
Rng::Rng() : generator(std::random_device{}()) {}

Rng::Rng(uint_fast32_t seed) : generator(seed) {}

double Rng::sample_normal_distribution(double mean, double stddev)
{
  std::normal_distribution<> distribution{mean, stddev};
  return distribution(generator);
}

std::function<double()> Rng::normal_distribution(double mean, double stddev)
{
  return [self = shared_from_this(), d = std::normal_distribution<>{mean, stddev}]() mutable {
    return d(self->generator);
  };
}

double Rng::sample_uniform_distribution(double a, double b)
{
  std::uniform_real_distribution<> distribution{a, b};
  return distribution(generator);
}

std::function<double()> Rng::uniform_distribution(double a, double b)
{
  return [self = shared_from_this(), d = std::uniform_real_distribution<>{a, b}]() mutable {
    return d(self->generator);
  };
}
}  // namespace ruvu_mcl
