// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include "../src/motion_models/differential_motion_model.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "tf2/utils.h"

constexpr double eps = 1e-15;

TEST(TestSuite, testTrans1)
{
  tf2::Transform delta{tf2::Quaternion::getIdentity(), tf2::Vector3{1, 0, 0}};
  auto [delta_rot1, delta_trans, delta_rot2] = DifferentialMotionModel::calculate_deltas(delta);
  ASSERT_NEAR(delta_rot1, 0, eps);
  ASSERT_NEAR(delta_trans, 1, eps);
  ASSERT_NEAR(delta_rot2, 0, eps);
}

TEST(TestSuite, testRotateInPlace)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, 1);
  tf2::Transform delta{q, tf2::Vector3{0, 0, 0}};

  auto [delta_rot1, delta_trans, delta_rot2] = DifferentialMotionModel::calculate_deltas(delta);
  ASSERT_NEAR(delta_rot1, 0, eps);
  ASSERT_NEAR(delta_trans, 0, eps);
  ASSERT_NEAR(delta_rot2, 1, eps);
}

TEST(TestSuite, testRotate)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_4);
  tf2::Transform delta{q, tf2::Vector3{1, 1, 0}};

  auto [delta_rot1, delta_trans, delta_rot2] = DifferentialMotionModel::calculate_deltas(delta);
  ASSERT_NEAR(delta_rot1, M_PI_4, eps);
  ASSERT_NEAR(delta_trans, M_SQRT2, eps);
  ASSERT_NEAR(delta_rot2, 0, eps);
}

TEST(TestSuite, testBackward)
{
  tf2::Transform delta{tf2::Quaternion::getIdentity(), tf2::Vector3{-1, 0, 0}};
  auto [delta_rot1, delta_trans, delta_rot2] = DifferentialMotionModel::calculate_deltas(delta);
  ASSERT_NEAR(delta_rot1, 0, eps);
  ASSERT_NEAR(delta_trans, -1, eps);
  ASSERT_NEAR(delta_rot2, 0, eps);
}

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
