// Copyright 2021 RUVU Robotics B.V.

#include <vector>

#include "../src/config.hpp"
#include "../src/filter.hpp"
#include "../src/particle_filter.hpp"
#include "gtest/gtest.h"
#include "tf2/utils.h"

class FilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int n = 100.0;
    double eps = 1e-6;
    for (int i = 0; i < n; ++i) {
      double x = i / (1.0 * n) + eps;
      double y = i / (2.0 * n) + eps;
      double weight = 1. / n;
      tf2::Quaternion q;
      q.setRPY(0, 0, i / (1.0 * n) + eps);
      pf.particles.emplace_back(tf2::Transform{q, tf2::Vector3{x, y, 0}}, double{weight});
    }
    pf.normalize_weights();

    // Initialize config
    config.min_particles = 10;
    config.max_particles = 150;
    adaptive_config.xy_grid_size = 0.1;
    adaptive_config.theta_grid_size = 0.05;
    adaptive_config.split_weight = 0.02;
    config.adaptive = adaptive_config;
  }

  Config config;
  SplitAndMergeConfig adaptive_config;
  ParticleFilter pf;
};

TEST_F(FilterTest, Merge)
{
  SplitAndMerge split_and_merge(config);
  split_and_merge.merge_particles(&pf);
  EXPECT_EQ(pf.particles.size(), 20);
  double total_weight = 0;
  for (auto & particle : pf.particles) total_weight += particle.weight;
  EXPECT_FLOAT_EQ(total_weight, 1.0);
  double max_weight = 0;
  for (auto & particle : pf.particles) {
    if (particle.weight > max_weight) max_weight = particle.weight;
  }
  EXPECT_FLOAT_EQ(max_weight, 0.05);
}

TEST_F(FilterTest, Split)
{
  SplitAndMerge split_and_merge(config);
  split_and_merge.merge_particles(&pf);
  split_and_merge.split_particles(&pf);
  EXPECT_EQ(pf.particles.size(), 60);
  double total_weight = 0;
  for (auto & particle : pf.particles) total_weight += particle.weight;
  EXPECT_FLOAT_EQ(total_weight, 1.0);
  double max_weight = 0;
  double min_weight = 1;
  for (auto & particle : pf.particles) {
    if (particle.weight > max_weight) max_weight = particle.weight;
    if (particle.weight < min_weight) min_weight = particle.weight;
  }
  EXPECT_FLOAT_EQ(max_weight, 0.05 / 3);
  EXPECT_FLOAT_EQ(min_weight, 0.05 / 3);
}

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
