#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pf/rv_samp.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <cstdio>
#include <iostream>

#include "./map.hpp"
#include "./motion_models/differential_motion_model.hpp"
#include "./sensor_models/beam_model.hpp"

class Gps
{
public:
  Gps(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : laser_scan_sub_(nh, "scan", 100),
    laser_scan_filter_(laser_scan_sub_, tf_buffer, "odom", 100, nh),
    map_sub_(nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Gps::map_cb, this)),
    cloud_pub_(private_nh.advertise<visualization_msgs::Marker>("cloud", 1)),
    laser_(std::make_unique<BeamModel>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 10, map_))
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
    publish_particle_cloud(scan->header.stamp);

    last_odom_pose_ = odom_pose;

    // TODO: convert scan to LaserData
    LaserData data;
    laser_->sensorUpdate(&particles_, data);
  }

  void map_cb(const nav_msgs::OccupancyGrid::ConstPtr & map)
  {
    ROS_INFO("map received");

    // TODO: convert OccupancyGrid to map
    // map_ = convert(map);
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

  void publish_particle_cloud(const ros::Time & time)
  {
    constexpr double length = 0.1;
    constexpr double width = 0.02;

    visualization_msgs::Marker m;
    m.header.stamp = time;
    m.header.frame_id = "map";
    m.ns = "particles";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::MODIFY;
    m.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
    m.scale.x = width;
    m.color.a = 1;
    m.color.b = 1;

    tf2::Vector3 p1{length / 2, 0, 0};
    for (const auto & particle : particles_) {
      {
        geometry_msgs::Point tmp;
        tf2::toMsg(particle.pose * p1, tmp);
        m.points.push_back(std::move(tmp));
      }
      {
        geometry_msgs::Point tmp;
        tf2::toMsg(particle.pose * -p1, tmp);
        m.points.push_back(std::move(tmp));
      }
    }

    cloud_pub_.publish(std::move(m));
  }

  // data input
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{tf_buffer};
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_scan_filter_;
  ros::Subscriber map_sub_;

  // data output
  ros::Publisher cloud_pub_;

  // internals
  tf2::Transform last_odom_pose_ = {};
  pf::rvsamp::UnivNormSampler<double> distribution{0, 0.1};
  ParticleFilter particles_;
  DifferentialMotionModel model_ = {0.1, 0.1, 0.1, 0.1};
  Map map_;
  std::unique_ptr<Laser> laser_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gps");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  Gps gps{nh, private_nh};
  ROS_INFO("%s started", private_nh.getNamespace().c_str());
  ros::spin();
  return EXIT_SUCCESS;
}