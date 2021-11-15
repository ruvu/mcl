// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include "../src/map.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/console.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using ruvu_mcl::Map;

TEST(TestSuite, test_world2map_offset)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.resolution = 0.1;  // m/pixel
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::toMsg(tf2::Transform{q, tf2::Vector3{10, 10, 0}}, msg.info.origin);
  Map map{msg};
  {
    auto [i, j] = map.world2map({10, 10, 0});
    ASSERT_EQ(i, 0);
    ASSERT_EQ(j, 0);
  }
  {
    auto [i, j] = map.world2map({9, 12, 0});
    ASSERT_EQ(i, 20);
    ASSERT_EQ(j, 10);
  }
}

TEST(TestSuite, test_world2map_identity)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.resolution = 0.1;  // m/pixel
  tf2::toMsg(tf2::Transform::getIdentity(), msg.info.origin);

  Map map{msg};
  {
    auto [i, j] = map.world2map({0, 0, 0});
    ASSERT_EQ(i, 0);
    ASSERT_EQ(j, 0);
  }
  {
    auto [i, j] = map.world2map({1, 2, 0});
    ASSERT_EQ(i, 10);
    ASSERT_EQ(j, 20);
  }
}

TEST(TestSuite, test_world2map_rounding)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.resolution = 1;  // m/pixel
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::toMsg(tf2::Transform::getIdentity(), msg.info.origin);
  Map map{msg};
  {
    auto [i, j] = map.world2map({0.1, 0, 0});
    ASSERT_EQ(i, 0);
  }
  {
    auto [i, j] = map.world2map({0.9, 0, 0});
    ASSERT_EQ(i, 1);
  }
  {
    auto [i, j] = map.world2map({1.1, 0, 0});
    ASSERT_EQ(i, 1);
  }
  {
    auto [i, j] = map.world2map({-0.1, 0, 0});
    ASSERT_EQ(i, 0);
  }
  {
    auto [i, j] = map.world2map({-0.9, 0, 0});
    ASSERT_EQ(i, -1);
  }
  {
    auto [i, j] = map.world2map({-1.1, 0, 0});
    ASSERT_EQ(i, -1);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
