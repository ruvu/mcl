// Copyright 2021 RUVU Robotics B.V.

#include "./filter.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "./adaptive/fixed.hpp"
#include "./adaptive/kld_sampling.hpp"
#include "./adaptive/split_and_merge.hpp"
#include "./cloud_publisher.hpp"
#include "./map.hpp"
#include "./motion_models/differential_motion_model.hpp"
#include "./resamplers/low_variance.hpp"
#include "./rng.hpp"
#include "./sensor_models/beam_model.hpp"
#include "./sensor_models/likelihood_field_model.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/UInt32.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

constexpr auto name = "filter";

Filter::Filter(
  ros::NodeHandle nh, ros::NodeHandle private_nh,
  const std::shared_ptr<const tf2_ros::Buffer> & buffer)
: buffer_(buffer),
  cloud_pub_(nh, private_nh),
  count_pub_(private_nh.advertise<std_msgs::UInt32>("count", 1)),
  pose_pub_(private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1)),
  config_(),
  rng_(std::make_unique<Rng>()),
  last_odom_pose_(),
  filter_(),
  model_(nullptr),
  map_(nullptr),
  lasers_(),
  resampler_(std::make_unique<LowVariance>(rng_)),
  resample_count_(0),
  adaptive_(nullptr)
{
}

void Filter::configure(const Config & config)
{
  ROS_INFO_NAMED(name, "configure call");
  config_ = config;

  lasers_.clear();  // they will configure themself on next scan_cb

  if (auto c = std::get_if<DifferentialMotionModelConfig>(&config.model))
    model_ = std::make_unique<DifferentialMotionModel>(*c, rng_);
  else
    throw std::logic_error("no motion model configured");

  if (std::holds_alternative<FixedConfig>(config.adaptive))
    adaptive_ = std::make_unique<Fixed>(config);
  else if (auto c = std::get_if<KLDSamplingConfig>(&config.adaptive))
    adaptive_ = std::make_unique<KLDSampling>(*c);
  else if (std::holds_alternative<SplitAndMergeConfig>(config.adaptive))
    adaptive_ = std::make_unique<SplitAndMerge>(config);
  else
    throw std::logic_error("no adaptive algorithm configured");

  if (filter_.particles.size() == 0) {
    auto p = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    p->header.stamp = ros::Time::now();
    p->header.frame_id = config_.global_frame_id;
    tf2::toMsg(config.initial_pose.getOrigin(), p->pose.pose.position);
    p->pose.pose.orientation = tf2::toMsg(config.initial_pose.getRotation());
    std::copy(config.initial_cov.begin(), config.initial_cov.end(), p->pose.covariance.begin());
    initial_pose_cb(p);
  }
}

Filter::~Filter() = default;

void Filter::scan_cb(const sensor_msgs::LaserScanConstPtr & scan)
{
  assert(model_);

  tf2::Transform odom_pose;
  try {
    odom_pose = get_odom_pose(scan->header.stamp);
  } catch (const tf2::TransformException & e) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return;
  }

  if (!last_odom_pose_) {
    ROS_INFO_NAMED(name, "first scan_cb, recording the odom pose");
    last_odom_pose_ = odom_pose;
    return;
  }

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

  assert(adaptive_);
  adaptive_->after_odometry_update(&filter_);

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

  adaptive_->after_sensor_update(&filter_);

  auto needed_particles = adaptive_->calc_needed_particles(&filter_);

  // Resample
  if (config_.selective_resampling) {
    if (filter_.calc_effective_sample_size() < filter_.particles.size() / 2)
      resampler_->resample(&filter_, needed_particles);
  } else {
    if (!(++resample_count_ % config_.resample_interval))
      resampler_->resample(&filter_, needed_particles);
  }

  // Create output
  std_msgs::Header header;
  header.stamp = scan->header.stamp;
  header.frame_id = config_.global_frame_id;
  cloud_pub_.publish(header, filter_);
  std_msgs::UInt32 count;
  count.data = filter_.particles.size();
  count_pub_.publish(count);
  geometry_msgs::PoseWithCovarianceStamped average_pose_with_covariance_stamped =
    filter_.get_pose_with_covariance_stamped(scan->header.stamp, config_.global_frame_id);
  tf2::convert(average_pose_with_covariance_stamped.pose.pose, last_pose_.emplace());
  pose_pub_.publish(std::move(average_pose_with_covariance_stamped));
  broadcast_tf(last_pose_.value(), last_odom_pose_.value(), scan->header.stamp);
}

void Filter::map_cb(const nav_msgs::OccupancyGridConstPtr & map)
{
  ROS_INFO_NAMED(name, "map received");
  map_ = map;
}

void Filter::initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose)
{
  auto & p = initial_pose->pose;
  ROS_INFO_NAMED(
    name, "initial pose received %.3f %.3f %.3f, spawning %lu new particles", p.pose.position.x,
    p.pose.position.y, tf2::getYaw(p.pose.orientation), config_.max_particles);

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

  cloud_pub_.publish(initial_pose->header, filter_);
  geometry_msgs::PoseWithCovarianceStamped average_pose_with_covariance_stamped =
    filter_.get_pose_with_covariance_stamped(initial_pose->header.stamp, config_.global_frame_id);
  tf2::convert(average_pose_with_covariance_stamped.pose.pose, last_pose_.emplace());
  pose_pub_.publish(std::move(average_pose_with_covariance_stamped));
}

tf2::Transform Filter::get_odom_pose(const ros::Time & time)
{
  geometry_msgs::Pose odom_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), odom_pose);

  // don't use .transform() because this could run offline without a listener thread
  auto tf = buffer_->lookupTransform(config_.odom_frame_id, config_.base_frame_id, time);
  tf2::doTransform(odom_pose, odom_pose, tf);

  tf2::Transform odom_pose_tf;
  tf2::convert(odom_pose, odom_pose_tf);
  return odom_pose_tf;
}

void Filter::broadcast_tf(
  const tf2::Transform pose, const tf2::Transform odom_pose, const ros::Time stamp)
{
  // Broadcast transform
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp + ros::Duration(config_.transform_tolerance);

  transform_msg.header.frame_id = config_.global_frame_id;
  transform_msg.child_frame_id = config_.odom_frame_id;
  transform_msg.transform = tf2::toMsg(pose * odom_pose.inverse());
  transform_br_.sendTransform(std::move(transform_msg));
}
