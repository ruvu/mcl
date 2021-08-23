// Copyright 2021 RUVU Robotics B.V.

#include <math.h>

#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

#include "./filesystem.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "interactive_markers/interactive_marker_server.h"
#include "ros/node_handle.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"
#include "visualization_msgs/Marker.h"

using json = nlohmann::json;

struct Landmark
{
  Landmark(const tf2::Transform & pose, int32_t id = 0) : pose(pose), id(id) {}
  int32_t id;
  tf2::Transform pose;
};

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

class Node
{
public:
  Node(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : landmarks_(), path_(), interactive_marker_server_("reflectors")
  {
    pose_sub_ =
      nh.subscribe<geometry_msgs::PoseStamped>("add_landmark", 1, &Node::add_landmark_cb, this);
    landmarks_pub_ = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmarks", 1, true);

    {
      std::string path;
      if (!private_nh.getParam("file_path", path))
        throw std::runtime_error("parameter 'file_path' does not exist");
      path_ = path;
    }

    if (!std::filesystem::exists(path_)) {
      ROS_INFO("no input file detected, file will be created on save");
    } else {
      std::ifstream file(path_);
      if (!file.is_open()) throw std::runtime_error("file_path can't be opened");
      json j = json::parse(file);
      auto landmarks = j.get<std::vector<Landmark>>();
      for (const auto & landmark : landmarks) {
        add_landmark(landmark);
      }
      ROS_INFO("successfuly loaded landmarks from %s", path_.c_str());
    }

    publish_landmarks(ros::Time::now());
  }

private:
  void add_landmark_cb(const geometry_msgs::PoseStampedConstPtr & msg)
  {
    ROS_INFO("add landmark callback");
    tf2::Vector3 position;
    tf2::Quaternion orientation;
    tf2::fromMsg(msg->pose.orientation, orientation);
    tf2::fromMsg(msg->pose.position, position);
    if (fabs(orientation.length2() - 1) > 1e-6) {
      ROS_WARN("input quaternion is not normalized, skipping pose");
      return;
    }
    add_landmark(tf2::Transform{orientation, position});
    save_landmarks();
    publish_landmarks(msg->header.stamp);
  }

  void add_landmark(Landmark landmark)
  {
    std::string landmark_name = "landmark_" + std::to_string(interactive_marker_server_.size());
    auto interactive_marker_ = create_interactive_marker(landmark, landmark_name);
    interactive_marker_server_.insert(interactive_marker_, boost::bind(&Node::marker_cb, this, _1));
    interactive_marker_server_.applyChanges();
    landmarks_.emplace(landmark_name, landmark);
  }

  void publish_landmarks(const ros::Time & stamp)
  {
    ruvu_mcl_msgs::LandmarkList msg;
    for (const auto & [_, landmark] : landmarks_) {
      ruvu_mcl_msgs::LandmarkEntry entry;
      entry.pose.pose.position.x =
        landmark.pose.getOrigin().getX();  // tf2::convert() fails for some reason
      entry.pose.pose.position.y = landmark.pose.getOrigin().getY();
      entry.pose.pose.position.z = landmark.pose.getOrigin().getZ();
      tf2::convert(landmark.pose.getRotation(), entry.pose.pose.orientation);
      msg.landmarks.push_back(std::move(entry));
    }
    msg.header.frame_id = "map";
    msg.header.stamp = stamp;
    landmarks_pub_.publish(msg);
  }

  void save_landmarks()
  {
    std::vector<Landmark> landmarks;
    for (const auto & [_, landmark] : landmarks_) landmarks.push_back(landmark);
    json j = landmarks;
    std::ofstream file(path_);
    file << std::setw(2) << j;
  }

  visualization_msgs::InteractiveMarker create_interactive_marker(
    Landmark landmark, std::string name)
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.pose.position.x =
      landmark.pose.getOrigin().getX();  // tf2::convert() fails for some reason
    int_marker.pose.position.y = landmark.pose.getOrigin().getY();
    int_marker.pose.position.z = landmark.pose.getOrigin().getZ();
    tf2::convert(landmark.pose.getRotation(), int_marker.pose.orientation);
    int_marker.scale = 0.15;
    int_marker.name = name;
    int_marker.description = "id: " + std::to_string(landmark.id);
    double l = 1 / sqrt(2);
    auto control = create_marker_control(
      "move_xy_plane", visualization_msgs::InteractiveMarkerControl::MOVE_PLANE,
      tf2::toMsg(tf2::Quaternion(0, l, 0, l)));
    auto marker_arrow = create_marker_arrow(int_marker.scale);
    control.markers.push_back(marker_arrow);
    control.always_visible = true;
    int_marker.controls.push_back(control);
    int_marker.controls.push_back(create_marker_control(
      "move_x", visualization_msgs::InteractiveMarkerControl::MOVE_AXIS,
      tf2::toMsg(tf2::Quaternion(l, 0, 0, l))));
    int_marker.controls.push_back(create_marker_control(
      "move_y", visualization_msgs::InteractiveMarkerControl::MOVE_AXIS,
      tf2::toMsg(tf2::Quaternion(0, 0, l, l))));
    int_marker.controls.push_back(create_marker_control(
      "rotate_z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS,
      tf2::toMsg(tf2::Quaternion(0, l, 0, l))));
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

  void marker_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
  {
    tf2::Vector3 position;
    tf2::Quaternion orientation;
    tf2::fromMsg(feedback->pose.orientation, orientation);
    tf2::fromMsg(feedback->pose.position, position);
    landmarks_.at(feedback->marker_name) = tf2::Transform{orientation, position};
    interactive_marker_server_.applyChanges();
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    {
      ROS_INFO_STREAM(
        feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                              << feedback->pose.position.y << ", " << feedback->pose.position.z);
      save_landmarks();
    }
  }

  ros::Subscriber pose_sub_;
  ros::Publisher landmarks_pub_;

  interactive_markers::InteractiveMarkerServer interactive_marker_server_;
  std::map<std::string, Landmark> landmarks_;
  std::filesystem::path path_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "landmark_server");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  auto node = Node{nh, private_nh};

  ros::spin();
  return EXIT_SUCCESS;
}
