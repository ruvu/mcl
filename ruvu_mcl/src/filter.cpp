// Copyright 2021 RUVU Robotics B.V.

#include "./filter.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "./map.hpp"
#include "./motion_models/differential_motion_model.hpp"
#include "./resamplers/low_variance.hpp"
#include "./rng.hpp"
#include "./sensor_models/beam_model.hpp"
#include "./sensor_models/likelihood_field_model.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/Marker.h"

constexpr auto name = "filter";

Filter::Filter(
  ros::NodeHandle nh, ros::NodeHandle private_nh,
  const std::shared_ptr<const tf2_ros::Buffer> & buffer)
: buffer_(buffer),
  cloud_pub_(private_nh.advertise<visualization_msgs::Marker>("cloud", 1)),
  pose_pub_(private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1)),
  config_(),
  rng_(std::make_unique<Rng>()),
  last_odom_pose_(),
  filter_(),
  model_(nullptr),
  map_(nullptr),
  lasers_(),
  resampler_(std::make_unique<LowVariance>(rng_)),
  resample_count_(0)
{
}

void Filter::configure(const Config & config)
{
  ROS_INFO_NAMED(name, "configure call");

  lasers_.clear();  // they will configure themself on next scan_cb

  if (auto c = std::get_if<DifferentialMotionModelConfig>(&config.model))
    model_ = std::make_unique<DifferentialMotionModel>(*c, rng_);
  else
    throw std::logic_error("no motion model configured");

  if (filter_.particles.size() == 0) {
    auto p = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    p->header.stamp = ros::Time::now();
    p->header.frame_id = config_.global_frame_id;
    p->pose.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
    p->pose.covariance[0 * 6 + 0] = 0.2;
    p->pose.covariance[1 * 6 + 1] = 0.2;
    p->pose.covariance[5 * 6 + 5] = 0.2;
    initial_pose_cb(p);
  }

  config_ = config;
}

Filter::~Filter() = default;

void Filter::scan_cb(const sensor_msgs::LaserScanConstPtr & scan)
{
  assert(model_);

  if (!last_odom_pose_) {
    ROS_INFO_NAMED(name, "first scan_cb, recording the odom pose");
    last_odom_pose_ = get_odom_pose(scan->header.stamp);
    return;
  }

  auto odom_pose = get_odom_pose(scan->header.stamp);
  auto diff = last_odom_pose_->inverseTimes(odom_pose);
  if (
    diff.getOrigin().length() < config_.update_min_d &&
    fabs(tf2::getYaw(diff.getRotation())) < config_.update_min_a) {
    if (last_pose_ && last_odom_pose_) {
      broadcast_tf(last_pose_.value(), last_odom_pose_.value(), scan->header.stamp);
    }
    return;
  }
  ROS_DEBUG_NAMED(
    name, "movement: x=%f y=%f t=%f", diff.getOrigin().getX(), diff.getOrigin().getY(),
    tf2::getYaw(diff.getRotation()));

  model_->odometry_update(&filter_, odom_pose, diff);

  last_odom_pose_ = odom_pose;

  if (!map_) {
    ROS_WARN_NAMED(name, "no map yet received, skipping sensor model");
    return;
  }
  if (lasers_.find(scan->header.frame_id) == lasers_.end()) {
    ROS_INFO_NAMED(
      name, "new laser '%s' found, adding a sensor model", scan->header.frame_id.c_str());

    // TODO(Ramon): A copy is made of the map for each sensor, but hey should all use the same
    // shared pointer
    std::unique_ptr<Laser> sensor;
    if (auto c = std::get_if<BeamModelConfig>(&config_.laser))
      sensor = std::make_unique<BeamModel>(*c, std::make_shared<OccupancyMap>(*map_));
    else if (auto c = std::get_if<LikelihoodFieldModelConfig>(&config_.laser))
      sensor = std::make_unique<LikelihoodFieldModel>(*c, std::make_shared<DistanceMap>(*map_));
    else
      throw std::logic_error("no laser model configured");
    lasers_.insert({scan->header.frame_id, std::move(sensor)});
  }

  tf2::Transform tf;
  {
    auto tfs =
      buffer_->lookupTransform(config_.base_frame_id, scan->header.frame_id, scan->header.stamp);
    tf2::fromMsg(tfs.transform, tf);
  }

  LaserData data(*scan, tf);
  lasers_.at(scan->header.frame_id)->sensor_update(&filter_, data);

  // Resample
  if (config_.selective_resampling) {
    if (filter_.calc_effective_sample_size() < filter_.particles.size() / 2)
      resampler_->resample(&filter_);
  } else {
    if (!(++resample_count_ % config_.resample_interval)) resampler_->resample(&filter_);
  }

  // Create output
  publish_particle_cloud(scan->header.stamp);
  last_pose_ = get_output_pose(filter_);
  publish_pose_with_covariance(last_pose_.value());
  broadcast_tf(last_pose_.value(), last_odom_pose_.value(), scan->header.stamp);
}

void Filter::map_cb(const nav_msgs::OccupancyGridConstPtr & map)
{
  ROS_INFO_NAMED(name, "map received");
  map_ = map;
}

void Filter::initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose)
{
  ROS_INFO_NAMED(name, "initial pose received");

  auto & p = initial_pose->pose;
  auto dx = rng_->normal_distribution(p.pose.position.x, p.covariance[0 * 6 + 0]);
  auto dy = rng_->normal_distribution(p.pose.position.y, p.covariance[1 * 6 + 1]);
  auto dt = rng_->normal_distribution(tf2::getYaw(p.pose.orientation), p.covariance[5 * 6 + 5]);

  filter_.particles.clear();
  for (size_t i = 0; i < config_.max_particles; ++i) {
    tf2::Vector3 p{dx(), dy(), 0};
    tf2::Quaternion q;
    q.setRPY(0, 0, dt());
    filter_.particles.emplace_back(tf2::Transform{q, p}, 1. / config_.max_particles);
  }
  publish_particle_cloud(initial_pose->header.stamp);
  last_pose_ = get_output_pose(filter_);
  publish_pose_with_covariance(last_pose_.value());
}

tf2::Transform Filter::get_odom_pose(const ros::Time & time)
{
  ROS_DEBUG_NAMED(name, "get_odom_pose at time=%f", time.toSec());
  geometry_msgs::Pose odom_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), odom_pose);

  // don't use .transform() because this could run offline without a listener thread
  auto tf = buffer_->lookupTransform(config_.odom_frame_id, config_.base_frame_id, time);
  tf2::doTransform(odom_pose, odom_pose, tf);

  tf2::Transform odom_pose_tf;
  tf2::convert(odom_pose, odom_pose_tf);
  return odom_pose_tf;
}

void Filter::publish_particle_cloud(const ros::Time & time)
{
  constexpr double length = 0.1;
  constexpr double width = 0.02;

  visualization_msgs::Marker m;
  m.header.stamp = time;
  m.header.frame_id = config_.global_frame_id;
  m.ns = "particles";
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.action = visualization_msgs::Marker::MODIFY;
  m.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
  m.scale.x = width;
  m.points.reserve(filter_.particles.size() * 2);
  m.colors.reserve(filter_.particles.size() * 2);

  double max_weight = 0;
  for (const auto & p : filter_.particles) max_weight = std::max(max_weight, p.weight);

  tf2::Vector3 p1{length / 2, 0, 0};
  for (const auto & particle : filter_.particles) {
    std_msgs::ColorRGBA c;
    c.a = particle.weight / max_weight;
    c.b = 1;
    m.colors.push_back(c);
    m.colors.push_back(std::move(c));
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

tf2::Transform Filter::get_output_pose(const ParticleFilter pf)
{
  // Find best particle
  Particle max_weight_particle(tf2::Transform::getIdentity(), 0);
  for (const auto & particle : filter_.particles) {
    if (particle.weight > max_weight_particle.weight) {
      max_weight_particle = particle;
    }
  }
  return max_weight_particle.pose;
}

void Filter::publish_pose_with_covariance(const tf2::Transform pose)
{
  // Publish PoseWithCovarianceStamped
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = config_.global_frame_id;

  tf2::toMsg(pose, pose_msg.pose.pose);
  auto cov = filter_.get_2d_covariance_array();
  assert(cov.size() == pose_msg.pose.covariance.size());
  std::copy(cov.begin(), cov.end(), pose_msg.pose.covariance.begin());

  pose_pub_.publish(std::move(pose_msg));
}

void Filter::broadcast_tf(
  const tf2::Transform pose, const tf2::Transform odom_pose, const ros::Time stamp)
{
  // Broadcast transform
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = config_.global_frame_id;
  transform_msg.child_frame_id = config_.odom_frame_id;
  transform_msg.transform = tf2::toMsg(pose * odom_pose.inverse());
  transform_br_.sendTransform(std::move(transform_msg));
}
