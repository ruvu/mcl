// Copyright 2021 RUVU Robotics B.V.

#include "../src/particle_filter.hpp"
#include "../src/rng.hpp"
#include "gtest/gtest.h"
#include "ros/init.h"
#include "tf2/utils.h"
#include "tf2_eigen/tf2_eigen.h"

class ParticleFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int n = 10;
    for (int i = 0; i < n; ++i) {
      double x = (2.0 * i) / n;  // [0, 2]
      double y = (4.0 * i) / n;  // [0, 4]
      double weight = 1;
      tf2::Quaternion q;
      q.setRPY(0, 0, (0.2 * i) / n);  // [0, 0.2]
      pf.particles.emplace_back(tf2::Transform{q, tf2::Vector3{x, y, 0}}, double{weight});
    }
  }

  ParticleFilter pf;
};

TEST_F(ParticleFilterTest, Normalize)
{
  double sum_of_weights = 0;
  for (auto & particle : pf.particles) sum_of_weights += particle.weight;
  EXPECT_NE(sum_of_weights, 1.0);
  pf.normalize_weights();
  sum_of_weights = 0;
  for (auto & particle : pf.particles) sum_of_weights += particle.weight;
  EXPECT_FLOAT_EQ(sum_of_weights, 1.0);
}

TEST_F(ParticleFilterTest, EffectiveSampleSize)
{
  pf.normalize_weights();
  double effective_size = pf.calc_effective_sample_size();
  int nr_particles = pf.particles.size();
  EXPECT_FLOAT_EQ(effective_size, nr_particles);
}

TEST_F(ParticleFilterTest, Covariance)
{
  pf.normalize_weights();
  auto cov = pf.get_2d_covariance_array();

  std::array<double, 4> sum_of_squares = {0};
  std::array<double, 4> sum = {0};
  for (auto & particle : pf.particles) {
    sum_of_squares[0] +=
      particle.weight * particle.pose.getOrigin().getX() * particle.pose.getOrigin().getX();  // XX
    sum_of_squares[1] +=
      particle.weight * particle.pose.getOrigin().getX() * particle.pose.getOrigin().getY();  // XY
    sum_of_squares[2] +=
      particle.weight * particle.pose.getOrigin().getY() * particle.pose.getOrigin().getX();  // YX
    sum_of_squares[3] +=
      particle.weight * particle.pose.getOrigin().getY() * particle.pose.getOrigin().getY();  // YY

    sum[0] += particle.weight * particle.pose.getOrigin().getX();
    sum[1] += particle.weight * particle.pose.getOrigin().getY();
    sum[2] += particle.weight * cos(tf2::getYaw(particle.pose.getRotation()));
    sum[3] += particle.weight * sin(tf2::getYaw(particle.pose.getRotation()));
  }

  std::array<double, 4> square_of_sum = {0};
  square_of_sum[0] += sum[0] * sum[0];  // XX
  square_of_sum[1] += sum[0] * sum[1];  // XY
  square_of_sum[2] += sum[1] * sum[0];  // YX
  square_of_sum[3] += sum[1] * sum[1];  // YY

  std::array<double, 4> expected_xy_cov;
  for (size_t i = 0; i < 4; i++) expected_xy_cov[i] = sum_of_squares[i] - square_of_sum[i];

  EXPECT_FLOAT_EQ(cov[0], expected_xy_cov[0]);
  EXPECT_FLOAT_EQ(cov[1], expected_xy_cov[1]);
  EXPECT_FLOAT_EQ(cov[6], expected_xy_cov[2]);
  EXPECT_FLOAT_EQ(cov[7], expected_xy_cov[3]);

  double expected_theta_cov = -2 * log(sqrt(sum[2] * sum[2] + sum[3] * sum[3]));
  EXPECT_FLOAT_EQ(cov[35], expected_theta_cov);
}

class ParticleFilterCovarianceTest : public ::testing::Test
{
protected:
  void SetUp(double mean, double stddev)
  {
    auto rng = std::make_shared<Rng>(4);
    auto d = rng->normal_distribution(mean, stddev);

    // generate a set of particles according to a normal distribution
    pf.particles.clear();
    int n = 1000;
    for (int i = 0; i < n; ++i) {
      tf2::Quaternion q;
      q.setRPY(0, 0, d());
      pf.particles.emplace_back(tf2::Transform{q, tf2::Vector3{d(), d(), 0}}, 1. / n);
    }
  }

  ParticleFilter pf;
};

TEST_F(ParticleFilterCovarianceTest, Covariance)
{
  SetUp(0, 0.5);
  constexpr double eps = 5e-2;

  auto cov = pf.get_2d_covariance_array();
  EXPECT_NEAR(cov[0], pow(0.5, 2), eps);
  EXPECT_NEAR(cov[1], 0, eps);
  EXPECT_NEAR(cov[6], 0, eps);
  EXPECT_NEAR(cov[7], pow(0.5, 2), eps);
}

TEST_F(ParticleFilterCovarianceTest, CovarianceWithOffset)
{
  SetUp(3, 0.5);
  constexpr double eps = 5e-2;

  auto cov = pf.get_2d_covariance_array();
  EXPECT_NEAR(cov[0], pow(0.5, 2), eps);
  EXPECT_NEAR(cov[1], 0, eps);
  EXPECT_NEAR(cov[6], 0, eps);
  EXPECT_NEAR(cov[7], pow(0.5, 2), eps);
}

TEST_F(ParticleFilterCovarianceTest, CovarianceTransformed)
{
  SetUp(0, 1);
  constexpr double eps = 5e-3;

  // scaling matrix
  tf2::Matrix3x3 s{0.2, 0, 0, 0, 0.5, 0, 0, 0, 0};

  // rotation matrix
  tf2::Quaternion q;
  q.setRPY(0, 0, 1);
  tf2::Matrix3x3 r{q};
  auto t = r * s;
  for (auto & p : pf.particles) {
    p.pose.setOrigin(t * p.pose.getOrigin());
  }

  // https://www.visiondummy.com/2014/04/geometric-interpretation-covariance-matrix/
  auto expected = r * s * s * r.transpose();

  auto cov = pf.get_2d_covariance_array();
  EXPECT_NEAR(cov[0], expected[0][0], eps);
  EXPECT_NEAR(cov[1], expected[0][1], eps);
  EXPECT_NEAR(cov[6], expected[1][0], eps);
  EXPECT_NEAR(cov[7], expected[1][1], eps);
}

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
