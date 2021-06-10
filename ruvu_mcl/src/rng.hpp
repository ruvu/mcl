// Copyright 2021 RUVU Robotics B.V.

#pragma once
#include <random>
class Rng
{
public:
  Rng();
  double sample_normal_distribution(double mean, double stddev = 1.0);
  double sample_uniform_distribution(double a, double b);

private:
  std::mt19937 generator;
};
