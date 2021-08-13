// Copyright 2021 RUVU Robotics B.V.

#include "./landmark.hpp"

#include "ruvu_mcl_msgs/LandmarkList.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
LandmarkList::LandmarkList(const ruvu_mcl_msgs::LandmarkList & msg) : landmarks()
{
  for (const auto & landmark : msg.landmarks) {
    tf2::Transform pose;
    tf2::convert(landmark.pose.pose, pose);
    landmarks.emplace_back(std::move(pose));
  }
}
