#include <gtest/gtest.h>

#include "../src/map.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

TEST(TestSuite, testCells)
{
  nav_msgs::OccupancyGrid msg;
  msg.info.width = 2;
  msg.info.height = 3;
  msg.data = {0, 100, -1, 100, 100, 100};
  /**
   * 1 2 3
   * 4 5 6
   */

  Map map{msg};
  ASSERT_EQ(map.cells.rows(), msg.info.width);
  ASSERT_EQ(map.cells.cols(), msg.info.height);

  // The map data is in row-major order, so let's check
  ASSERT_EQ(map.cells(0, 0), -1);
  ASSERT_EQ(map.cells(0, 1), 1);
  ASSERT_EQ(map.cells(0, 2), 0);
  ASSERT_EQ(map.cells(1, 0), 1);
  ASSERT_EQ(map.cells(1, 1), 1);
  ASSERT_EQ(map.cells(1, 2), 1);
}

TEST(TestSuite, testOrigin)
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

TEST(TestSuite, testOrigin2)
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

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
