// Copyright 2021 RUVU Robotics B.V.

#include "./interactive_markers.hpp"

#include <math.h>

#include <string>

#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"
#include "visualization_msgs/Marker.h"

visualization_msgs::InteractiveMarker create_interactive_marker(
  tf2::Transform landmark_pose, int landmark_id, std::string name)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf2::toMsg(landmark_pose.getOrigin(), int_marker.pose.position);
  tf2::convert(landmark_pose.getRotation(), int_marker.pose.orientation);
  int_marker.scale = 0.15;
  int_marker.name = name;
  int_marker.description = name + ", id: " + (landmark_id ? std::to_string(landmark_id) : "<none>");
  auto control = create_marker_control(
    "move_xy_plane", visualization_msgs::InteractiveMarkerControl::MOVE_PLANE,
    tf2::toMsg(tf2::Quaternion(0, 1, 0, 1)));
  auto marker_arrow = create_marker_arrow(int_marker.scale);
  control.markers.push_back(marker_arrow);
  control.always_visible = true;
  int_marker.controls.push_back(control);
  int_marker.controls.push_back(create_marker_control(
    "move_x", visualization_msgs::InteractiveMarkerControl::MOVE_AXIS,
    tf2::toMsg(tf2::Quaternion(1, 0, 0, 1))));
  int_marker.controls.push_back(create_marker_control(
    "move_y", visualization_msgs::InteractiveMarkerControl::MOVE_AXIS,
    tf2::toMsg(tf2::Quaternion(0, 0, 1, 1))));
  int_marker.controls.push_back(create_marker_control(
    "rotate_z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS,
    tf2::toMsg(tf2::Quaternion(0, 1, 0, 1))));
  return int_marker;
}

visualization_msgs::InteractiveMarkerControl create_marker_control(
  std::string control_name, int control_mode, geometry_msgs::Quaternion orientation)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation = orientation;
  control.name = control_name;
  control.interaction_mode = control_mode;
  return control;
}

visualization_msgs::Marker create_marker_arrow(double marker_scale)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = marker_scale * 2;
  marker.scale.y = marker_scale / 5;
  marker.scale.z = marker_scale / 5;
  marker.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  return marker;
}
