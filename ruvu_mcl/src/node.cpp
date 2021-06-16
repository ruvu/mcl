// Copyright 2021 RUVU Robotics B.V.

#include "./node.hpp"

#include <memory>
#include <utility>

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

constexpr auto name = "node";

Node::Node(ros::NodeHandle nh, ros::NodeHandle private_nh)
: laser_scan_sub_(nh, "scan", 100),
  laser_scan_filter_(laser_scan_sub_, tf_buffer, "odom", 100, nh),
  map_sub_(nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, &Node::map_cb, this)),
  initial_pose_sub_(nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
    "initialpose", 1, &Node::initial_pose_cb, this)),
  cloud_pub_(private_nh.advertise<visualization_msgs::Marker>("cloud", 1)),
  pose_pub_(private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1)),
  rng_(std::make_unique<Rng>()),
  filter_(),
  model_(std::make_unique<DifferentialMotionModel>(0.1, 0.1, 0.1, 0.1, rng_)),
  lasers_(),
  resampler_(std::make_unique<LowVariance>(rng_)),
  resample_count_(0)
{
  last_odom_pose_ = get_odom_pose(ros::Time{0}, ros::Duration{5});

  laser_scan_filter_.registerCallback(&Node::scan_cb, this);

  int n = 10;
  for (int i = 0; i < n; ++i) {
    tf2::Quaternion q;
    q.setRPY(0, 0, rng_->sample_normal_distribution(0, 0.2));
    tf2::Vector3 p{
      rng_->sample_normal_distribution(0, 0.2), rng_->sample_normal_distribution(0, 0.2), 0};
    filter_.particles.emplace_back(tf2::Transform{q, p}, 1. / n);
  }
}

Node::~Node() = default;

void Node::scan_cb(const sensor_msgs::LaserScanConstPtr & scan)
{
  auto odom_pose = get_odom_pose(scan->header.stamp);
  auto diff = last_odom_pose_.inverseTimes(odom_pose);

  double update_min_d = 0.25;
  double update_min_a = 0.2;
  if (
    diff.getOrigin().length() < update_min_d &&
    fabs(tf2::getYaw(diff.getRotation())) < update_min_a) {
    publish_particle_cloud(scan->header.stamp);
    return;
  }
  ROS_DEBUG_NAMED(
    name, "movement: x=%f y=%f t=%f", diff.getOrigin().getX(), diff.getOrigin().getY(),
    tf2::getYaw(diff.getRotation()));

  model_->odometry_update(&filter_, odom_pose, diff);
  publish_particle_cloud(scan->header.stamp);

  last_odom_pose_ = odom_pose;

  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    ROS_INFO_NAMED(
      name, "new laser '%s' found, adding a sensor model", scan->header.frame_id.c_str());
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

  LaserData data(*scan, tf);
  lasers_.at(scan->header.frame_id)->sensor_update(&filter_, data);

  // Resample
  bool selective_resampling = true;
  int resample_interval = 2;
  if (selective_resampling) {
    if (filter_.calc_effective_sample_size() < filter_.particles.size() / 2)
      resampler_->resample(&filter_);
  } else {
    if (!(++resample_count_ % resample_interval)) resampler_->resample(&filter_);
  }

  // Find best particle
  Particle max_weight_particle(tf2::Transform::getIdentity(), 0);
  for (const auto & particle : filter_.particles) {
    if (particle.weight > max_weight_particle.weight) {
      max_weight_particle = particle;
    }
  }

  // Publish PoseWithCovarianceStamped
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "map";
  filter_.get_2d_covariance_array(&pose_msg.pose.covariance[0]);
  tf2::toMsg(max_weight_particle.pose, pose_msg.pose.pose);
  pose_pub_.publish(std::move(pose_msg));

  // Broadcast transform
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header = pose_msg.header;
  transform_msg.child_frame_id = "odom";
  transform_msg.transform = tf2::toMsg(max_weight_particle.pose * odom_pose.inverse());
  transform_br_.sendTransform(std::move(transform_msg));
}

void Node::map_cb(const nav_msgs::OccupancyGridConstPtr & map)
{
  ROS_INFO_NAMED(name, "map received");
  if (!map_)
    map_ = std::make_unique<Map>(*map);
  else
    *map_ = Map{*map};
}

void Node::initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose)
{
  ROS_INFO_NAMED(name, "initial pose received");

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

  int n = 10;
  filter_.particles.clear();
  for (int i = 0; i < n; ++i) {
    tf2::Vector3 p{dx(), dy(), 0};
    tf2::Quaternion q;
    q.setRPY(0, 0, dt());
    filter_.particles.emplace_back(tf2::Transform{q, p}, 1. / n);
  }
}

tf2::Transform Node::get_odom_pose(const ros::Time & time, ros::Duration timeout)
{
  geometry_msgs::PoseStamped odom_pose;
  odom_pose.header.stamp = time;
  odom_pose.header.frame_id = "base_link";
  tf2::toMsg(tf2::Transform::getIdentity(), odom_pose.pose);
  tf_buffer.transform(odom_pose, odom_pose, "odom", timeout);

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
  for (const auto & particle : filter_.particles) {
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
