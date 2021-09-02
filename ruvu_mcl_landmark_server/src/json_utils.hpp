// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <nlohmann/json.hpp>

#include "./landmark.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using json = nlohmann::json;

namespace nlohmann
{
template <>
struct adl_serializer<tf2::Transform>
{
  static void to_json(json & j, const tf2::Transform & transform)
  {
    auto & p = transform.getOrigin();
    auto q = transform.getRotation();
    j = json{
      {"x", p.x()},  {"y", p.y()},  {"z", p.z()},  {"qx", q.x()},
      {"qy", q.y()}, {"qz", q.z()}, {"qw", q.w()},
    };
  }

  static void from_json(const json & j, tf2::Transform & transform)
  {
    transform.getOrigin().setX(j.at("x").get<double>());
    transform.getOrigin().setY(j.at("y").get<double>());
    transform.getOrigin().setZ(j.at("z").get<double>());

    tf2::Quaternion q;
    q.setX(j.at("qx").get<double>());
    q.setY(j.at("qy").get<double>());
    q.setZ(j.at("qz").get<double>());
    q.setW(j.at("qw").get<double>());
    transform.setRotation(q);
  }
};

template <>
struct adl_serializer<Landmark>
{
  static void to_json(json & j, const Landmark & landmark)
  {
    j = json{{"id", landmark.id}, {"pose", landmark.pose}};
  }

  static Landmark from_json(const json & j)
  {
    return Landmark{j.at("pose").get<tf2::Transform>(), j.at("id").get<int32_t>()};
  }
};
}  // namespace nlohmann
