#include "landmark_display.hpp"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/validate_floats.h>
#include <rviz/validate_quaternions.h>

namespace ruvu_mcl_rviz_plugins
{
Ogre::Vector3 vectorRosToOgre(geometry_msgs::Point const & point)
{
  return Ogre::Vector3(point.x, point.y, point.z);
}

Ogre::Quaternion quaternionRosToOgre(geometry_msgs::Quaternion const & quaternion)
{
  Ogre::Quaternion q;
  rviz::normalizeQuaternion(quaternion, q);
  return q;
}

bool validateFloats(const ruvu_mcl_msgs::LandmarkList::ConstPtr & msg)
{
  bool valid = true;
  for (const auto landmark : msg->landmarks) {
    valid = valid && rviz::validateFloats(landmark.pose.pose);
    valid = valid && rviz::validateFloats(landmark.pose.covariance);
    if (!valid) break;
  }
  return valid;
}

bool validateQuaternions(const ruvu_mcl_msgs::LandmarkList::ConstPtr & msg)
{
  bool valid = true;
  for (const auto landmark : msg->landmarks) {
    valid = valid && rviz::validateQuaternions(landmark.pose.pose);
    if (!valid) break;
  }
  return valid;
}

LandmarkDisplay::LandmarkDisplay()
{
  axes_length_property_ = new rviz::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.", this, SLOT(updateAxisGeometry()), this);

  axes_radius_property_ = new rviz::FloatProperty(
    "Axes Radius", 0.1, "Radius of each axis, in meters.", this, SLOT(updateAxisGeometry()), this);
}

void LandmarkDisplay::onInitialize() { MFDClass::onInitialize(); }

void LandmarkDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void LandmarkDisplay::updateAxisGeometry()
{
  for (auto it = visuals_.begin(); it != visuals_.end(); ++it) {
    (*it)->set(axes_length_property_->getFloat(), axes_radius_property_->getFloat());
  }
  context_->queueRender();
}

bool LandmarkDisplay::setTransform(std_msgs::Header const & header)
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(header, position, orientation)) {
    ROS_ERROR(
      "Error transforming pose '%s' from frame '%s' to frame '%s'", qPrintable(getName()),
      header.frame_id.c_str(), qPrintable(fixed_frame_));
    return false;
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
  return true;
}

void LandmarkDisplay::processMessage(const ruvu_mcl_msgs::LandmarkList::ConstPtr & msg)
{
  if (!validateFloats(msg)) {
    setStatus(
      rviz::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  if (!validateQuaternions(msg)) {
    ROS_WARN_ONCE_NAMED(
      "quaternions",
      "LandmarkList '%s' contains unnormalized quaternions. "
      "This warning will only be output once but may be true for others; "
      "enable DEBUG messages for ros.rviz.quaternions to see more details.",
      qPrintable(getName()));
    ROS_DEBUG_NAMED(
      "quaternions", "LandmarkList '%s' contains unnormalized quaternions.", qPrintable(getName()));
  }

  if (!setTransform(msg->header)) {
    setStatus(rviz::StatusProperty::Error, "Topic", "Failed to look up transform");
    return;
  }

  visuals_.resize(msg->landmarks.size());
  for (std::size_t i = 0; i < msg->landmarks.size(); ++i) {
    visuals_[i]->setPosition(vectorRosToOgre(msg->landmarks[i].pose.pose.position));
    visuals_[i]->setOrientation(quaternionRosToOgre(msg->landmarks[i].pose.pose.orientation));
  }
}

}  // namespace ruvu_mcl_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ruvu_mcl_rviz_plugins::LandmarkDisplay, rviz::Display)
