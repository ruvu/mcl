// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <vector>

#include "ros/message_forward.h"
#include "std_msgs/Header.h"
#include "tf2/LinearMath/Transform.h"

namespace sensor_msgs
{
ROS_DECLARE_MESSAGE(LaserScan)
}
namespace ruvu_mcl_msgs
{
ROS_DECLARE_MESSAGE(LandmarkList)
}

namespace ruvu_mcl
{
// forward declare
class ParticleFilter;

/**
 * @brief The a single Landmark measurment
 */
struct Landmark
{
  explicit Landmark(const tf2::Transform & pose) : pose(pose), id(0) {}
  Landmark(const tf2::Transform & pose, int id) : pose(pose), id(id) {}

  /**
   * @brief Pose of the landmark
   *
   * Landmarks that are from the landmark map are assumed to be transformed to the global frame.
   * Landmark from measurements are assumed to be in the sensor frame.
   */
  tf2::Transform pose;

  /**
   * @brief optional identity of the landmark. If the id is 0, it has no identity.
   */
  int id;
};

/**
 * @brief Landmark measurment data
 *
 * LandmarkList is used in two cases. One as a map of landmarks. Two as Landmark measurment data.
 */
class LandmarkList
{
public:
  /**
   * @brief Create an empty LandmarkList
   */
  LandmarkList() : header(), pose(tf2::Transform::getIdentity()), landmarks() {}

  /**
   * @brief Create a landmark list from a ruvu_mcl_msgs::LandmarkList
   * @param msg ruvu_mcl_msgs::LandmarkList
   * @param pose For landmark measurments, the pose of the sensor relative to the base_link must be
   * given.
   */
  explicit LandmarkList(
    const ruvu_mcl_msgs::LandmarkList & msg,
    const tf2::Transform & pose = tf2::Transform::getIdentity());

  /**
   * @brief timestamp in the header is the acquisition time of the measurement
   */
  std_msgs::Header header;

  /**
   * @brief LandmarkLists from measurements should contain a pose with the transform from robot frame to sensor frame.
   */
  tf2::Transform pose;

  /**
   * @brief List of all detected landmarks
   */
  std::vector<Landmark> landmarks;
};

/**
 * @brief Base class for landmark sensor models
 *
 * Base class for all landmark sensor models. A sensor model tries to accurately model the specific
 * types of uncertainty that exist in a sensor.
 *
 * Sensor models take a measurment as input and update the weight of the particles according to how
 * good the position estimate explains the sensor measurment. See also chapter 6 of Probabilistc
 * Robotics.
 */
class LandmarkModel
{
public:
  virtual ~LandmarkModel();

  /**
   * @brief Run a sensor update on the particles with landmark data
   * @param pf Particle filter to use
   * @param data Landmark data to use
   */
  virtual void sensor_update(ParticleFilter * pf, const LandmarkList & data) = 0;
};
}  // namespace ruvu_mcl
