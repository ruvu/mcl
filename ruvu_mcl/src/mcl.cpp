// Copyright 2021 RUVU Robotics B.V.

#include "./mcl.hpp"

#include <algorithm>
#include <memory>

#include "./adaptive/fixed.hpp"
#include "./adaptive/kld_sampling.hpp"
#include "./adaptive/split_and_merge.hpp"
#include "./map.hpp"
#include "./motion_models/differential_motion_model.hpp"
#include "./resamplers/low_variance.hpp"
#include "./rng.hpp"
#include "./sensor_models/beam_model.hpp"
#include "./sensor_models/gaussian_landmark_model.hpp"
#include "./sensor_models/landmark_likelihood_field_model.hpp"
#include "./sensor_models/likelihood_field_model.hpp"
#include "ros/node_handle.h"
#include "std_msgs/UInt32.h"
#include "tf2/utils.h"

constexpr auto name = "mcl";

std::unique_ptr<Laser> create_laser_model(
  const Config & config, const nav_msgs::OccupancyGridConstPtr & map)
{
  ROS_INFO_NAMED(name, "adding a laser sensor model");
  if (const auto * c = std::get_if<BeamModelConfig>(&config.laser))
    return std::make_unique<BeamModel>(*c, std::make_shared<OccupancyMap>(*map));
  else if (const auto * c = std::get_if<LikelihoodFieldModelConfig>(&config.laser))
    return std::make_unique<LikelihoodFieldModel>(*c, std::make_shared<DistanceMap>(*map));
  else
    throw std::logic_error("no laser model configured");
}

Mcl::Mcl(ros::NodeHandle nh, ros::NodeHandle private_nh)
: cloud_pub_(nh, private_nh),
  count_pub_(private_nh.advertise<std_msgs::UInt32>("count", 1)),
  pose_pub_(private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1)),
  config_(),
  rng_(std::make_unique<Rng>()),
  last_odom_pose_(),
  filter_(),
  model_(nullptr),
  map_(nullptr),
  landmarks_(nullptr),
  laser_(nullptr),
  landmark_model_(nullptr),
  should_process_(),
  resampler_(std::make_unique<LowVariance>(rng_)),
  resample_count_(0),
  adaptive_(nullptr)
{
}

void Mcl::configure(const Config & config)
{
  ROS_INFO_NAMED(name, "configure call");
  config_ = config;

  // they will configure themself on next scan_cb
  laser_ = nullptr;
  landmark_model_ = nullptr;

  if (const auto * c = std::get_if<DifferentialMotionModelConfig>(&config.model))
    model_ = std::make_unique<DifferentialMotionModel>(*c, rng_);
  else
    throw std::logic_error("no motion model configured");

  if (std::holds_alternative<FixedConfig>(config.adaptive))
    adaptive_ = std::make_unique<Fixed>(config);
  else if (const auto * c = std::get_if<KLDSamplingConfig>(&config.adaptive))
    adaptive_ = std::make_unique<KLDSampling>(*c);
  else if (std::holds_alternative<SplitAndMergeConfig>(config.adaptive))
    adaptive_ = std::make_unique<SplitAndMerge>(config);
  else
    throw std::logic_error("no adaptive algorithm configured");

  if (filter_.particles.empty()) {
    auto p = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
    p->header.stamp = ros::Time::now();
    p->header.frame_id = config_.global_frame_id;
    tf2::toMsg(config.initial_pose.getOrigin(), p->pose.pose.position);
    p->pose.pose.orientation = tf2::toMsg(config.initial_pose.getRotation());
    std::copy(config.initial_cov.begin(), config.initial_cov.end(), p->pose.covariance.begin());
    initial_pose_cb(p);
  }
}

Mcl::~Mcl() = default;

bool Mcl::scan_cb(const LaserData & scan, const tf2::Transform & odom_pose)
{
  if (!odometry_update(scan.header, MeasurementType::LASER, odom_pose)) return false;

  assert(adaptive_);
  adaptive_->after_odometry_update(&filter_);

  if (!map_) {
    ROS_WARN_NAMED(name, "no map yet received, skipping sensor model");
    return false;
  }
  if (!laser_) {
    laser_ = create_laser_model(config_, map_);
  }

  laser_->sensor_update(&filter_, scan);

  adaptive_->after_sensor_update(&filter_);

  auto needed_particles = adaptive_->calc_needed_particles(filter_);

  // Resample
  if (config_.selective_resampling) {
    if (filter_.calc_effective_sample_size() < filter_.particles.size() / 2.)
      resampler_->resample(&filter_, needed_particles);
  } else {
    if (!(++resample_count_ % config_.resample_interval))
      resampler_->resample(&filter_, needed_particles);
  }

  // Create output
  auto ps = filter_.get_pose_with_covariance_stamped(scan.header.stamp, config_.global_frame_id);
  publish_data(ps);
  tf2::Transform pose;
  tf2::fromMsg(ps.pose.pose, pose);
  last_pose_.setData(pose);  // store last_pose_ for later use
  last_pose_.stamp_ = scan.header.stamp;
  return true;
}

bool Mcl::landmark_cb(const LandmarkList & landmarks, const tf2::Transform & odom_pose)
{
  if (!odometry_update(landmarks.header, MeasurementType::LANDMARK, odom_pose)) return false;

  assert(adaptive_);
  adaptive_->after_odometry_update(&filter_);

  if (!landmarks_) {
    ROS_WARN_NAMED(name, "no landmark list yet received, skipping sensor model");
    return false;
  }
  if (!landmark_model_) {
    ROS_INFO_NAMED(name, "adding a landmark sensor model");
    if (const auto * c = std::get_if<GaussianLandmarkModelConfig>(&config_.landmark))
      landmark_model_ = std::make_unique<GaussianLandmarkModel>(*c, *landmarks_);
    else if (const auto * c = std::get_if<LandmarkLikelihoodFieldModelConfig>(&config_.landmark))
      landmark_model_ = std::make_unique<LandmarkLikelihoodFieldModel>(*c, *landmarks_);
    else
      throw std::logic_error("no landmark model configured");
  }

  landmark_model_->sensor_update(&filter_, landmarks);

  adaptive_->after_sensor_update(&filter_);

  auto needed_particles = adaptive_->calc_needed_particles(filter_);

  // Resample
  if (config_.selective_resampling) {
    if (filter_.calc_effective_sample_size() < filter_.particles.size() / 2.)
      resampler_->resample(&filter_, needed_particles);
  } else {
    if (!(++resample_count_ % config_.resample_interval))
      resampler_->resample(&filter_, needed_particles);
  }

  // Create output
  auto ps =
    filter_.get_pose_with_covariance_stamped(landmarks.header.stamp, config_.global_frame_id);
  publish_data(ps);
  tf2::Transform pose;
  tf2::fromMsg(ps.pose.pose, pose);
  last_pose_.setData(pose);  // store last_pose_ for later use
  last_pose_.stamp_ = landmarks.header.stamp;
  return true;
}

void Mcl::map_cb(const nav_msgs::OccupancyGridConstPtr & map)
{
  ROS_INFO_NAMED(name, "map received");
  map_ = map;
  laser_ = nullptr;
}

void Mcl::landmark_list_cb(const LandmarkList & landmarks)
{
  ROS_INFO_NAMED(name, "landmark list received");
  landmarks_ = std::make_shared<LandmarkList>(landmarks);
  landmark_model_ = nullptr;
}

void Mcl::initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose)
{
  const auto & p = initial_pose->pose;
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

  auto ps =
    filter_.get_pose_with_covariance_stamped(initial_pose->header.stamp, config_.global_frame_id);
  publish_data(ps);
  tf2::Transform pose;
  tf2::fromMsg(ps.pose.pose, pose);
  last_pose_.setData(pose);  // store last_pose_ for later use
  last_pose_.stamp_ = initial_pose->header.stamp;
}

void Mcl::request_nomotion_update()
{
  for (auto & key : should_process_) {
    key.second = true;
  }
}

bool Mcl::odometry_update(
  const std_msgs::Header & header, const MeasurementType & measurement_type,
  tf2::Transform odom_pose)
{
  assert(model_);

  if (!last_odom_pose_) {
    ROS_INFO_NAMED(name, "first odometry update, recording the odom pose");
    last_odom_pose_ = odom_pose;
    last_pose_.stamp_ = header.stamp;
    // call should_process to record the frame_id
    should_process(tf2::Transform::getIdentity(), {measurement_type, header.frame_id});
    return true;
  }

  if (header.stamp <= last_pose_.stamp_) {
    ROS_DEBUG_STREAM_NAMED(
      name, "skipping out-of-order measurement, " << header.stamp << " <= " << last_pose_.stamp_);
    return false;
  }

  auto diff = last_odom_pose_->inverseTimes(odom_pose);

  if (!should_process(diff, {measurement_type, header.frame_id})) {
    return false;
  }

  ROS_DEBUG_NAMED(
    name, "movement: x=%f y=%f t=%f", diff.getOrigin().getX(), diff.getOrigin().getY(),
    tf2::getYaw(diff.getRotation()));

  model_->odometry_update(&filter_, diff);

  last_odom_pose_ = odom_pose;
  return true;
}

bool Mcl::should_process(const tf2::Transform & diff, const MeasurementKey & measurment_key)
{
  if (
    diff.getOrigin().length() >= config_.update_min_d ||
    fabs(tf2::getYaw(diff.getRotation())) >= config_.update_min_a) {
    ROS_DEBUG_NAMED(name, "enough movement detected, processing all sensors");
    for (auto & s : should_process_) {
      s.second = true;
    }
  }

  if (should_process_.find(measurment_key) == should_process_.end()) {
    should_process_[measurment_key] = true;
  }

  if (should_process_[measurment_key]) {
    ROS_DEBUG_STREAM_NAMED(
      name, "processing " << std::get<0>(measurment_key) << ' ' << std::get<1>(measurment_key));
    should_process_[measurment_key] = false;
    return true;
  } else {
    return false;
  }
}

void Mcl::publish_data(const geometry_msgs::PoseWithCovarianceStamped & ps)
{
  cloud_pub_.publish(ps.header, filter_);
  std_msgs::UInt32 count;
  count.data = filter_.particles.size();
  count_pub_.publish(count);
  pose_pub_.publish(ps);
}

std::ostream & operator<<(std::ostream & out, const Mcl::MeasurementType & measurement_type)
{
  switch (measurement_type) {
    case Mcl::MeasurementType::LASER:
      return out << "laser";
    case Mcl::MeasurementType::LANDMARK:
      return out << "landmark";
    default:
      throw std::logic_error("unknown measurement type");
  }
}
