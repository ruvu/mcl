// Copyright 2021 RUVU Robotics B.V.

#include "./node.hpp"

#include "./map.hpp"
#include "./motion_models/differential_motion_model.hpp"
#include "./resamplers/low_variance.hpp"
#include "./rng.hpp"
#include "./sensor_models/beam_model.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/node_handle.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/utils.h"
#include "visualization_msgs/Marker.h"

Node::Node(ros::NodeHandle nh, ros::NodeHandle private_nh)
: laser_scan_sub_(nh, "scan", 100),
  laser_scan_filter_(laser_scan_sub_, tf_buffer, "odom", 100, nh),
  map_sub_(nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Node::map_cb, this)),
  initial_pose_sub_(nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    "initialpose", 1, &Node::initial_pose_cb, this)),
  cloud_pub_(private_nh.advertise<visualization_msgs::Marker>("cloud", 1)),
  rng_(std::make_unique<Rng>()),
  last_odom_pose_(),
  particles_(),
  model_(std::make_unique<DifferentialMotionModel>(0.1, 0.1, 0.1, 0.1, rng_)),
  lasers_(),
  resampler_(std::make_unique<LowVariance>(rng_))
{
  laser_scan_filter_.registerCallback(&Node::scan_cb, this);

  int n = 10;
  for (int i = 0; i < n; ++i) {
    tf2::Quaternion q;
    q.setRPY(0, 0, rng_->sample_normal_distribution(0, 0.2));
    tf2::Vector3 p{
      rng_->sample_normal_distribution(0, 0.2), rng_->sample_normal_distribution(0, 0.2), 0};
    particles_.emplace_back(tf2::Transform{q, p}, 1. / n);
  }
}

Node::~Node() = default;

void Node::scan_cb(const sensor_msgs::LaserScanConstPtr & scan)
{
  // TODO: skip the first odom update
  auto odom_pose = get_odom_pose(scan->header.stamp);
  auto diff = last_odom_pose_.inverseTimes(odom_pose);
  ROS_INFO(
    "diff: %f %f %f", diff.getOrigin().getX(), diff.getOrigin().getY(),
    tf2::getYaw(diff.getRotation()));

  model_->odometry_update(&particles_, odom_pose, diff);
  publish_particle_cloud(scan->header.stamp);

  last_odom_pose_ = odom_pose;

  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    ROS_INFO("new laser found, adding a sensor model");
    BeamModel::Parameters parameters;
    parameters.lambda_short = 0.1;
    parameters.sigma_hit = 0.2;
    parameters.z_hit = 0.5;
    parameters.z_max = 0.05;
    parameters.z_rand = 0.5;
    parameters.z_short = 0.05;
    parameters.max_beams = 60;
    auto sensor = std::make_unique<BeamModel>(parameters, map_);
    lasers_.insert({scan->header.frame_id, std::move(sensor)});
  }

  tf2::Transform tf;
  {
    auto tfs = tf_buffer.lookupTransform("base_link", scan->header.frame_id, scan->header.stamp);
    tf2::fromMsg(tfs.transform, tf);
  }
  {
    auto & p = tf.getOrigin();
    ROS_INFO("laser is mounted at %f %f %f", p.getX(), p.getY(), tf2::getYaw(tf.getRotation()));
  }
  LaserData data(*scan, tf);
  lasers_.at(scan->header.frame_id)->sensor_update(&particles_, data);

  resampler_->resample(&particles_);
}

void Node::map_cb(const nav_msgs::OccupancyGridConstPtr & map)
{
  ROS_INFO("map received");
  if (!map_)
    map_ = std::make_unique<Map>(*map);
  else
    *map_ = Map{*map};
}

void Node::initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose)
{
  ROS_INFO("initial pose received");

  auto & p = initial_pose->pose;
  auto dx = [this, &p]() {
    return rng_->sample_normal_distribution(p.pose.position.x, p.covariance[0 * 6 + 0]);
  };
  auto dy = [this, &p]() {
    return rng_->sample_normal_distribution(p.pose.position.y, p.covariance[1 * 6 + 1]);
  };
  auto dt = [this, &p]() {
    return rng_->sample_normal_distribution(
      tf2::getYaw(p.pose.orientation), p.covariance[5 * 6 + 5]);
  };

  particles_.clear();
  int n = 10;
  for (int i = 0; i < n; ++i) {
    tf2::Vector3 p{dx(), dy(), 0};
    tf2::Quaternion q;
    q.setRPY(0, 0, dt());
    particles_.emplace_back(tf2::Transform{q, p}, 1. / n);
  }
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
