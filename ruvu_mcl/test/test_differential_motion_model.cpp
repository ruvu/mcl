#include <gtest/gtest.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include "../src/motion_models/differential_motion_model.hpp"
#include "tf2/utils.h"

constexpr double eps = 1e-15;

// Declare a test
TEST(TestSuite, testTrans1)
{
  tf2::Transform delta{tf2::Quaternion::getIdentity(), tf2::Vector3{1, 0, 0}};
  auto [delta_rot1, delta_trans, delta_rot2] = DifferentialMotionModel::calculate_deltas(delta);
  ASSERT_NEAR(delta_rot1, 0, eps);
  ASSERT_NEAR(delta_trans, 1, eps);
  ASSERT_NEAR(delta_rot2, 0, eps);
}

// Declare a test
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

// Declare a test
TEST(TestSuite, testRotate)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::Transform delta{q, tf2::Vector3{0, 1, 0}};

  auto [delta_rot1, delta_trans, delta_rot2] = DifferentialMotionModel::calculate_deltas(delta);
  ASSERT_NEAR(delta_rot1, M_PI_2, eps);
  ASSERT_NEAR(delta_trans, 1, eps);
  ASSERT_NEAR(delta_rot2, 0, eps);
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
