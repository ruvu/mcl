// Copyright 2021 RUVU Robotics B.V.

#pragma once
#include <functional>
#include <memory>
#include <random>

class Rng : public std::enable_shared_from_this<Rng>
{
public:
  Rng();

  /**
   * @brief sample a normal_distribution once
   */
  double sample_normal_distribution(double mean, double stddev = 1.0);

  /**
   * @brief return a function to sample a normal_distribution
   */
  std::function<double()> normal_distribution(double mean, double stddev = 1.0);

  /**
   * @brief sample a uniform_distribution once
   */
  double sample_uniform_distribution(double a, double b);

  /**
   * @brief return a function to sample a uniform_distribution
   */
  std::function<double()> uniform_distribution(double a, double b = 1.0);

private:
  std::mt19937 generator;
};
