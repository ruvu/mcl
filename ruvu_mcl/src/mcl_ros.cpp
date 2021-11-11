// Copyright 2021 RUVU Robotics B.V.

#include "./mcl_ros.hpp"

#include <algorithm>
#include <memory>

#include "./sensor_models/landmark.hpp"
#include "./sensor_models/laser.hpp"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/UInt32.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"

constexpr auto name = "mcl_ros";

namespace ruvu_mcl
{
MclRos::MclRos(
  ros::NodeHandle nh, ros::NodeHandle private_nh,
  const std::shared_ptr<const tf2_ros::Buffer> & buffer)
: mcl_(nh, private_nh),
  buffer_(buffer),
  transform_br_(),
  last_tf_broadcast_(),
  cloud_pub_(nh, private_nh),
  count_pub_(private_nh.advertise<std_msgs::UInt32>("count", 1)),
  pose_pub_(private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1))
{
}

MclRos::~MclRos() = default;

void MclRos::configure(const ruvu_mcl::AMCLConfig & config) { mcl_.configure(Config{config}); }

void MclRos::scan_cb(const sensor_msgs::LaserScanConstPtr & scan)
{
  tf2::Transform tf;
  {
    auto tfs = buffer_->lookupTransform(
      mcl_.config().base_frame_id, scan->header.frame_id, scan->header.stamp);
    tf2::fromMsg(tfs.transform, tf);
  }
  LaserData data{*scan, tf};

  tf2::Transform odom_pose;
  try {
    odom_pose = get_odom_pose(data.header.stamp);
  } catch (const tf2::TransformException & e) {
    ROS_WARN_NAMED(name, "failed to compute odom pose, skipping measurement (%s)", e.what());
    return;
  }

  auto updated = mcl_.scan_cb(data, odom_pose);

  if (updated) {
    auto ps = mcl_.get_pose_with_covariance();
    broadcast_tf(ps.pose, odom_pose, data.header.stamp);
    publish_data(data.header.stamp, ps);
  } else if (!last_tf_broadcast_.header.frame_id.empty()) {  // Verify last_tf_broadcast_ is set
    broadcast_last_tf(data.header.stamp);
  }
}

void MclRos::landmark_cb(const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks)
{
  tf2::Transform tf;
  {
    auto tfs = buffer_->lookupTransform(
      mcl_.config().base_frame_id, landmarks->header.frame_id, landmarks->header.stamp);
    tf2::fromMsg(tfs.transform, tf);
  }
  LandmarkList data{*landmarks, tf};

  tf2::Transform odom_pose;
  try {
    odom_pose = get_odom_pose(data.header.stamp);
  } catch (const tf2::TransformException & e) {
    ROS_WARN_NAMED(name, "failed to compute odom pose, skipping measurement (%s)", e.what());
    return;
  }

  auto updated = mcl_.landmark_cb(data, odom_pose);

  if (updated) {
    auto ps = mcl_.get_pose_with_covariance();
    broadcast_tf(ps.pose, odom_pose, data.header.stamp);
    publish_data(data.header.stamp, ps);
  } else if (!last_tf_broadcast_.header.frame_id.empty()) {  // Verify last_tf_broadcast_ is set
    broadcast_last_tf(data.header.stamp);
  }
}

void MclRos::map_cb(const nav_msgs::OccupancyGridConstPtr & map) { mcl_.map_cb(map); }

void MclRos::landmark_list_cb(const ruvu_mcl_msgs::LandmarkListConstPtr & landmarks)
{
  LandmarkList data{*landmarks};
  mcl_.landmark_list_cb(data);
}

void MclRos::initial_pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr & initial_pose)
{
  if (initial_pose->header.frame_id != mcl_.config().global_frame_id) {
    ROS_WARN_NAMED(name, "initial pose is only accepted in the global frame");
  }

  // convert PoseWithCovariance to MCL datatype
  tf2::Transform p;
  tf2::convert(initial_pose->pose.pose.position, p.getOrigin());
  tf2::Quaternion q;
  tf2::convert(initial_pose->pose.pose.orientation, q);
  p.setRotation(q);
  std::array<double, 36> covariance;
  assert(initial_pose->pose.covariance.size() == covariance.size());
  std::copy(
    initial_pose->pose.covariance.begin(), initial_pose->pose.covariance.end(), covariance.begin());
  PoseWithCovariance pose_with_covariance{p, covariance};

  mcl_.initial_pose_cb(initial_pose->header.stamp, pose_with_covariance);
  mcl_.request_nomotion_update();
  publish_data(initial_pose->header.stamp, pose_with_covariance);
}

tf2::Transform MclRos::get_odom_pose(const ros::Time & time) const
{
  // don't use .transform() because this could run offline without a listener thread
  auto tf =
    buffer_->lookupTransform(mcl_.config().odom_frame_id, mcl_.config().base_frame_id, time);
  tf2::Transform odom_pose_tf;
  tf2::fromMsg(tf.transform, odom_pose_tf);
  return odom_pose_tf;
}

void MclRos::broadcast_tf(
  const tf2::Transform & pose, const tf2::Transform & odom_pose, const ros::Time & stamp)
{
  if (stamp <= last_tf_broadcast_.header.stamp) {
    return;
  }

  // Broadcast transform
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp + ros::Duration(mcl_.config().transform_tolerance);

  transform_msg.header.frame_id = mcl_.config().global_frame_id;
  transform_msg.child_frame_id = mcl_.config().odom_frame_id;
  transform_msg.transform = tf2::toMsg(pose * odom_pose.inverse());

  transform_br_.sendTransform(transform_msg);
  last_tf_broadcast_ = transform_msg;
}

void MclRos::broadcast_last_tf(const ros::Time & stamp)
{
  auto msg = last_tf_broadcast_;
  msg.header.stamp = stamp + ros::Duration(mcl_.config().transform_tolerance);

  transform_br_.sendTransform(msg);
  last_tf_broadcast_.header.stamp = stamp;
}

void MclRos::publish_data(const ros::Time & stamp, const PoseWithCovariance & pose_with_covariance)
{
  std_msgs::Header header;
  header.stamp = stamp;
  header.frame_id = mcl_.config().global_frame_id;
  cloud_pub_.publish(header, mcl_.particles());

  std_msgs::UInt32 count;
  count.data = mcl_.particles().size();
  count_pub_.publish(count);

  geometry_msgs::PoseWithCovarianceStamped ps;
  ps.header = header;
  tf2::toMsg(pose_with_covariance.pose.getOrigin(), ps.pose.pose.position);
  tf2::convert(pose_with_covariance.pose.getRotation(), ps.pose.pose.orientation);
  assert(pose_with_covariance.covariance.size() == ps.pose.covariance.size());
  std::copy(
    pose_with_covariance.covariance.begin(), pose_with_covariance.covariance.end(),
    ps.pose.covariance.begin());
  pose_pub_.publish(ps);
}
}  // namespace ruvu_mcl
