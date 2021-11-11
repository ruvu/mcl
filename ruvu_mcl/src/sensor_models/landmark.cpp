// Copyright 2021 RUVU Robotics B.V.

#include "./landmark.hpp"

#include "ruvu_mcl_msgs/LandmarkList.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace ruvu_mcl
{
LandmarkList::LandmarkList(const ruvu_mcl_msgs::LandmarkList & msg, const tf2::Transform & pose)
: header(msg.header), pose(pose), landmarks()
{
  for (const auto & landmark : msg.landmarks) {
    tf2::Transform pose;
    tf2::convert(landmark.pose.pose, pose);
    landmarks.emplace_back(Landmark(pose, landmark.id));
  }
}

LandmarkModel::~LandmarkModel() = default;
}  // namespace ruvu_mcl
