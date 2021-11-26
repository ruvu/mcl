// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include "ros/console.h"

// Copyright 2021 RUVU Robotics B.V.

#include <boost/filesystem/operations.hpp>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "../src/mcl_ros.hpp"
#include "../src/offline/bag_buffer.hpp"
#include "../src/offline/bag_player.hpp"
#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ruvu_mcl/AMCLConfig.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_msgs/TFMessage.h"

using nlohmann::json;
using ruvu_mcl::AMCLConfig;
using ruvu_mcl::BagBuffer;
using ruvu_mcl::BagPlayer;
using ruvu_mcl::MclRos;

constexpr auto name = "offline";

std::string quoted(const std::string & s)
{
  std::ostringstream ss;
  ss << std::quoted(s);
  return ss.str();
}

namespace nlohmann
{
template <>
struct adl_serializer<geometry_msgs::Point>
{
  static void to_json(json & j, const geometry_msgs::Point & value)
  {
    j["x"] = value.x;
    j["y"] = value.y;
    j["z"] = value.z;
  }
};

template <>
struct adl_serializer<geometry_msgs::Quaternion>
{
  static void to_json(json & j, const geometry_msgs::Quaternion & value)
  {
    j["x"] = value.x;
    j["y"] = value.y;
    j["z"] = value.z;
    j["w"] = value.w;
  }
};

template <>
struct adl_serializer<geometry_msgs::Pose>
{
  static void to_json(json & j, const geometry_msgs::Pose & value)
  {
    j["position"] = value.position;
    j["orientation"] = value.orientation;
  }
};

template <>
struct adl_serializer<geometry_msgs::PoseWithCovariance>
{
  static void to_json(json & j, const geometry_msgs::PoseWithCovariance & value)
  {
    j["pose"] = value.pose;
    j["covariance"] = value.covariance;
  }
};

}  // namespace nlohmann

TEST(RegressionTest, large_landmark_map)
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};

  auto path = ros::package::getPath("ruvu_mcl");
  path += "/test/large_landmark_map.bag";
  ROS_INFO_STREAM_NAMED(name, "opening " << path);
  BagPlayer player{path};
  player.set_playback_speed(std::numeric_limits<double>::infinity());

  auto buffer = std::make_shared<BagBuffer>(player.bag);
  MclRos filter{nh, private_nh, buffer, 1234};

  filter.configure(AMCLConfig::__getDefault__());

  json events;

  player.register_callback<ruvu_mcl_msgs::LandmarkList>(
    "/landmarks", [&filter, &events](const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks) {
      auto updated = filter.landmark_cb(landmarks);
      if (updated) {
        auto pose = filter.get_pose_with_covariance();
        events.push_back(pose);
      } else {
        events.push_back(updated);
      }
    });

  player.register_callback<ruvu_mcl_msgs::LandmarkList>(
    "/landmark_list", [&filter](const ruvu_mcl_msgs::LandmarkListConstPtr & landmark_list) {
      filter.landmark_list_cb(landmark_list);
    });

  player.register_callback<geometry_msgs::PoseWithCovarianceStamped>(
    "/initialpose",
    [&filter](const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose) {
      filter.initial_pose_cb(initial_pose);
    });

  player.start_play();

  // create a copy because mkstemp wants to modify its argument
  std::unique_ptr<char> tmpname{strdup(("/tmp" + private_nh.getNamespace() + "XXXXXX").c_str())};
  ASSERT_NE(-1, mkstemp(tmpname.get()));
  {
    // write the results to a temporary file
    std::ofstream ofs{tmpname.get()};
    ofs << std::setw(2) << events;
  }

  // compare the results with the diff command
  auto json_path = path + ".json";
  auto exit_code =
    std::system(("diff --unified " + ::quoted(json_path) + " " + tmpname.get()).c_str());
  ASSERT_EQ(0, exit_code) << "Output of the regression test at '" << tmpname.get()
                          << "' did not match '" << json_path << '\'';
}

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "regression_test_large_landmark_map");
  return RUN_ALL_TESTS();
}
