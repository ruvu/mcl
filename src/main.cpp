#include <message_filters/subscriber.h>
#include <pf/rv_samp.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <cstdio>
#include <iostream>

#include "motion_models/differential_motion_model.hpp"

using Input = Eigen::Matrix<double, 2, 1>;

static constexpr double g = 9.81;

class Model
{
public:
  double height = 0;
  double velocity = 0;

  void update(double dt)
  {
    velocity += -g * dt;
    height += velocity * dt;
  }

private:
};

class Gps
{
public:
  Gps(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : laser_scan_sub_(nh, "scan", 100),
    laser_scan_filter_(laser_scan_sub_, tf_buffer, "odom", 100, nh)
  {
    laser_scan_filter_.registerCallback(&Gps::scan_cb, this);

    for (int i = 0; i < 10; ++i) {
      particles_.emplace_back();
    }
  }

private:
  void scan_cb(const sensor_msgs::LaserScan::ConstPtr & scan)
  {
    // TODO: skip the first odom update
    auto odom_pose = get_odom_pose(scan->header.stamp);
    auto diff = last_odom_pose_.inverseTimes(odom_pose);
    ROS_INFO(
      "diff: %f %f %f", diff.getOrigin().getX(), diff.getOrigin().getY(),
      tf2::getYaw(diff.getRotation()));

    last_odom_pose_ = odom_pose;
  }

  tf2::Transform get_odom_pose(const ros::Time & time)
  {
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header.stamp = time;
    odom_pose.header.frame_id = "base_link";
    tf2::toMsg(tf2::Transform::getIdentity(), odom_pose.pose);
    tf_buffer.transform(odom_pose, odom_pose, "odom");

    ROS_INFO("%f %f", odom_pose.pose.position.x, odom_pose.pose.position.y);
    tf2::Stamped<tf2::Transform> odom_pose_tf;
    tf2::convert(odom_pose, odom_pose_tf);
    return odom_pose_tf;
  }

  // collecting all the data
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{tf_buffer};
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_scan_filter_;

  // internals
  tf2::Transform last_odom_pose_ = {};
  pf::rvsamp::UnivNormSampler<double> distribution{0, 0.1};
  std::vector<Particle> particles_;
  Model model_ = {};
  ros::Subscriber scan_sub_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gps");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_INFO("hello world");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  Gps gps{nh, private_nh};
  ros::spin();
  return EXIT_SUCCESS;
}
