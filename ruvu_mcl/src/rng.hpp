// Copyright 2021 RUVU Robotics B.V.

#pragma once
#include <functional>
#include <memory>
#include <random>

namespace ruvu_mcl
{
/**
 * @brief Utility class for generating random numbers
 */
class Rng : public std::enable_shared_from_this<Rng>
{
public:
  /**
   * @brief Construct Rng class with a random seed
   */
  Rng();

  /**
   * @brief Construct Rng class with a fixed seed
   *
   * This means that the class produces random numbers that are deterministic. Two classes with the
   * same seed produce the same sequence of random numbers. This can be used for example in CI to
   * create test cases that deterministic (i.e. not flaky).
   */
  explicit Rng(uint_fast32_t seed);

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
}  // namespace ruvu_mcl
