// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include "../src/map.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/console.h"

using ruvu_mcl::DistanceMap;

constexpr double eps = 1e-15;

TEST(TestSuite, test1)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 3;
  msg.info.height = 2;
  msg.info.resolution = 1;
  msg.data = {0, 0, 0, 0, 0, 100};
  //          0, 0, 0, 0, 0, 1
  /**
   * This is the storage order of an OccupancyGrid
   *   y (cols, height) ->
   * x 0 3
   *   1 4
   *   2 5
   */

  DistanceMap map{msg};
  ROS_INFO_STREAM("resulting distance map:\n" << map.cells);
  ASSERT_NEAR(map.cells(0, 0), sqrt(5), eps);
  ASSERT_NEAR(map.cells(1, 0), sqrt(2), eps);
  ASSERT_NEAR(map.cells(2, 0), 1, eps);
  ASSERT_NEAR(map.cells(0, 1), 2, eps);
  ASSERT_NEAR(map.cells(1, 1), 1, eps);
  ASSERT_NEAR(map.cells(2, 1), 0, eps);
}

TEST(TestSuite, test2)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 3;
  msg.info.height = 2;
  msg.info.resolution = 1;
  msg.data = {0, 0, 0, 0, 100, 100};
  //          0, 0, 0, 0,   1,   1
  /**
   * This is the storage order of an OccupancyGrid
   *   y (cols, height) ->
   * x 0 3
   *   1 4
   *   2 5
   */

  DistanceMap map{msg};
  ROS_INFO_STREAM("resulting distance map:\n" << map.cells);
  ASSERT_NEAR(map.cells(0, 0), sqrt(2), eps);
  ASSERT_NEAR(map.cells(1, 0), 1, eps);
  ASSERT_NEAR(map.cells(2, 0), 1, eps);
  ASSERT_NEAR(map.cells(0, 1), 1, eps);
  ASSERT_NEAR(map.cells(1, 1), 0, eps);
  ASSERT_NEAR(map.cells(2, 1), 0, eps);
}

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
