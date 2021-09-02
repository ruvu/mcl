// Copyright 2021 RUVU Robotics B.V.

#include <math.h>

#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

#include "./filesystem.hpp"
#include "./interactive_markers.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "interactive_markers/interactive_marker_server.h"
#include "ros/node_handle.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"

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
  : landmarks_(), path_(), interactive_marker_server_("reflectors"), next_landmark_number_(0)
  {
    landmarks_pub_ = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmarks", 1, true);
    add_landmark_sub_ =
      nh.subscribe<geometry_msgs::PoseStamped>("add_landmark", 1, &Node::addLandmarkCB, this);
    remove_landmark_sub_ = nh.subscribe<geometry_msgs::PointStamped>(
      "remove_landmark", 1, &Node::removeLandmarkCB, this);

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
        addLandmark(landmark);
      }
      ROS_INFO("successfuly loaded landmarks from %s", path_.c_str());
    }

    publishLandmarks(ros::Time::now());
  }

private:
  void addLandmarkCB(const geometry_msgs::PoseStampedConstPtr & msg)
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
    addLandmark(tf2::Transform{orientation, position});
    saveLandmarks();
    publishLandmarks(msg->header.stamp);
  }

  void addLandmark(Landmark landmark)
  {
    std::string landmark_name = "landmark_" + std::to_string(next_landmark_number_);
    next_landmark_number_++;
    auto interactive_marker_ = create_interactive_marker(landmark.pose, landmark.id, landmark_name);
    interactive_marker_server_.insert(interactive_marker_, boost::bind(&Node::markerCB, this, _1));
    interactive_marker_server_.applyChanges();
    landmarks_.emplace(landmark_name, landmark);
  }

  void removeLandmarkCB(const geometry_msgs::PointStampedConstPtr & msg)
  {
    tf2::Vector3 point;
    tf2::fromMsg(msg->point, point);
    std::vector<std::pair<std::string, Landmark>> nearby_landmarks;
    for (const auto & [landmark_name, landmark] : landmarks_) {
      if (tf2::tf2Distance(point, landmark.pose.getOrigin()) < 1)
        nearby_landmarks.push_back(make_pair(landmark_name, landmark));
    }
    if (nearby_landmarks.size() == 0) return;
    std::sort(
      begin(nearby_landmarks), end(nearby_landmarks),
      [point](
        const std::pair<std::string, Landmark> & lhs,
        const std::pair<std::string, Landmark> & rhs) {
        return tf2::tf2Distance2(point, lhs.second.pose.getOrigin()) <
               tf2::tf2Distance2(point, rhs.second.pose.getOrigin());
      });
    landmarks_.erase(nearby_landmarks[0].first);
    saveLandmarks();
    publishLandmarks(msg->header.stamp);
    interactive_marker_server_.erase(nearby_landmarks[0].first);
    interactive_marker_server_.applyChanges();
    ROS_INFO_STREAM("Removed " << nearby_landmarks[0].first);
  }

  void publishLandmarks(const ros::Time & stamp)
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

  void saveLandmarks()
  {
    std::vector<Landmark> landmarks;
    for (const auto & [_, landmark] : landmarks_) landmarks.push_back(landmark);
    json j = landmarks;
    std::ofstream file(path_);
    file << std::setw(2) << j;
  }

  void markerCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
  {
    tf2::Vector3 position;
    tf2::Quaternion orientation;
    tf2::fromMsg(feedback->pose.orientation, orientation);
    tf2::fromMsg(feedback->pose.position, position);
    landmarks_.at(feedback->marker_name) = tf2::Transform{orientation, position};
    interactive_marker_server_.applyChanges();
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      ROS_INFO_STREAM(
        feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                              << feedback->pose.position.y << ", " << feedback->pose.position.z);
      saveLandmarks();
    }
  }

  ros::Subscriber add_landmark_sub_;
  ros::Subscriber remove_landmark_sub_;
  ros::Publisher landmarks_pub_;
  uint next_landmark_number_;

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
