// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include "../src/map.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

TEST(TestSuite, testCells)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 3;
  msg.info.height = 2;
  msg.data = {0, 100, -1, 100, 100, 100};
  //          -1,  1,  0,   1,   1,   1
  /**
   * This is the storage order of an OccupancyGrid
   *   y (cols, height) ->
   * x 0 3
   *   1 4
   *   2 5
   */

  Map map{msg};

  std::cout << "In memory storage:\n";
  for (int i = 0; i < map.cells.size(); i++)
    std::cout << static_cast<int>(map.cells.data()[i]) << "  ";
  std::cout << '\n';

  ASSERT_EQ(map.cells.rows(), msg.info.width);
  ASSERT_EQ(map.cells.cols(), msg.info.height);

  // Let's check the map data
  ASSERT_EQ(map.cells(0, 0), -1);
  ASSERT_EQ(map.cells(1, 0), 1);
  ASSERT_EQ(map.cells(2, 0), 0);
  ASSERT_EQ(map.cells(0, 1), 1);
  ASSERT_EQ(map.cells(1, 1), 1);
  ASSERT_EQ(map.cells(2, 1), 1);
}

TEST(TestSuite, test_is_valid)
{
  // set the origin with an offset and turned 90 degree
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 3;
  msg.info.height = 2;
  msg.data.resize(msg.info.width * msg.info.height);

  Map map{msg};
  ASSERT_TRUE(map.is_valid(0, 0));
  ASSERT_TRUE(map.is_valid(2, 1));  // the corner
  ASSERT_FALSE(map.is_valid(3, 1));
  ASSERT_FALSE(map.is_valid(2, 2));
  ASSERT_FALSE(map.is_valid(-1, 0));
  ASSERT_FALSE(map.is_valid(0, -1));
}

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

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
