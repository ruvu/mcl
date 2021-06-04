#include <gtest/gtest.h>

#include "../src/map.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"

TEST(TestSuite, testMap)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 2;
  msg.info.height = 3;
  msg.data = {1, 2, 3, 4, 5, 6};
  /**
   * 1 2 3
   * 4 5 6
   */

  Map map{msg};
  ASSERT_EQ(map.cells.rows(), msg.info.width);
  ASSERT_EQ(map.cells.cols(), msg.info.height);

  // The map data is in row-major order, so let's check
  ASSERT_EQ(map.cells(0, 0), 1);
  ASSERT_EQ(map.cells(0, 1), 2);
  ASSERT_EQ(map.cells(0, 2), 3);
  ASSERT_EQ(map.cells(1, 0), 4);
  ASSERT_EQ(map.cells(1, 1), 5);
  ASSERT_EQ(map.cells(1, 2), 6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
