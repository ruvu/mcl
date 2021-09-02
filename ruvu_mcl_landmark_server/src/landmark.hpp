// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

struct Landmark
{
  Landmark(const tf2::Transform & pose) : pose(pose), id(0) {}
  Landmark(const tf2::Transform & pose, int32_t id) : pose(pose), id(id) {}
  int32_t id;
  tf2::Transform pose;
};
