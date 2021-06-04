#include "./node.hpp"

#include "ros/node_handle.h"
#include "tf2/utils.h"

Node::Node(ros::NodeHandle nh, ros::NodeHandle private_nh)
: laser_scan_sub_(nh, "scan", 100),
  laser_scan_filter_(laser_scan_sub_, tf_buffer, "odom", 100, nh),
  map_sub_(nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Node::map_cb, this)),
  cloud_pub_(private_nh.advertise<visualization_msgs::Marker>("cloud", 1)),
  laser_(std::make_unique<BeamModel>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 10, map_))
{
  laser_scan_filter_.registerCallback(&Node::scan_cb, this);

  for (int i = 0; i < 10; ++i) {
    particles_.emplace_back();
  }
}

void Node::scan_cb(const sensor_msgs::LaserScan::ConstPtr & scan)
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

void Node::map_cb(const nav_msgs::OccupancyGrid::ConstPtr & map)
{
  ROS_INFO("map received");
  map_ = Map{*map};
}

tf2::Transform Node::get_odom_pose(const ros::Time & time)
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

void Node::publish_particle_cloud(const ros::Time & time)
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
