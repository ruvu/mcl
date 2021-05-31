#include <message_filters/subscriber.h>
#include <pf/rv_samp.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <cstdio>
#include <iostream>

#include "motion_models/differential_motion_model.hpp"

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

    model_.odometry_update(&particles_, odom_pose, diff);
    publish_particle_cloud();

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

  void publish_particle_cloud()
  {
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.resize(particles_.size());
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::PointField::FLOAT64, "y", 1, sensor_msgs::PointField::FLOAT64, "z", 1,
      sensor_msgs::PointField::FLOAT64, "i", 1, sensor_msgs::PointField::INT8);
    sensor_msgs::PointCloud2Iterator<double> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<double> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<double> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<char> i(cloud, "i");

    for (const auto & particle : particles_) {
      auto & p = particle.pose.getOrigin();
      auto q = particle.pose.getRotation();
      *x = p.getX();
      *y = p.getY();
      *z = p.getZ();
      *i = 1;
      ++x;
      ++y;
      ++z;
      ++i;
    }
  }

  // collecting all the data
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{tf_buffer};
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_scan_filter_;

  // internals
  tf2::Transform last_odom_pose_ = {};
  pf::rvsamp::UnivNormSampler<double> distribution{0, 0.1};
  ParticleFilter particles_;
  DifferentialMotionModel model_ = {0.1, 0.1, 0.1, 0.1};
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
