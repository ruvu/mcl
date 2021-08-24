// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"
#include "visualization_msgs/Marker.h"

visualization_msgs::InteractiveMarker create_interactive_marker(
  tf2::Transform landmark_pose, int landmark_id, std::string name);
visualization_msgs::InteractiveMarkerControl create_marker_control(
  std::string control_name, int control_mode, geometry_msgs::Quaternion orientation);
visualization_msgs::Marker create_marker_arrow(double marker_scale);
