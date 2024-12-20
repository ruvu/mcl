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
#include "./json_utils.hpp"
#include "./landmark.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "interactive_markers/interactive_marker_server.h"
#include "ros/node_handle.h"
#include "ruvu_mcl_msgs/AddLandmark.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/InteractiveMarkerFeedback.h"

class Node
{
public:
  Node(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : landmarks_(),
    path_(),
    interactive_marker_server_("reflectors"),
    next_landmark_number_(0),
    frame_id_("map")
  {
    private_nh.param("max_landmark_remove_dist", max_landmark_remove_dist_, 1.0);

    landmarks_pub_ = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmark_list", 1, true);
    add_landmark_server_ = nh.advertiseService("add_landmark", &Node::addLandmarkCB, this);
    change_landmark_id_sub_ = nh.subscribe<ruvu_mcl_msgs::LandmarkEntry>(
      "change_landmark_id", 1, &Node::changeLandmarkIdCB, this);
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
  bool addLandmarkCB(
    ruvu_mcl_msgs::AddLandmarkRequest & req, ruvu_mcl_msgs::AddLandmarkResponse & res)  // NOLINT
  {
    if (req.header.frame_id != frame_id_) {
      ROS_ERROR_STREAM(
        "Landmarks must be added in the frame of the landmark server, which is " << frame_id_);
      return false;
    }
    tf2::Transform pose;
    tf2::fromMsg(req.landmark.pose.pose, pose);
    Landmark landmark(pose, req.landmark.id);
    return addLandmark(landmark);
  }

  bool addLandmark(Landmark landmark)
  {
    std::string landmark_name = "landmark_" + std::to_string(next_landmark_number_);
    next_landmark_number_++;
    auto interactive_marker_ = create_interactive_marker(landmark.pose, landmark.id, landmark_name);
    interactive_marker_server_.insert(interactive_marker_, boost::bind(&Node::markerCB, this, _1));
    interactive_marker_server_.applyChanges();
    landmarks_.emplace(landmark_name, landmark);
    return true;
  }

  std::vector<std::pair<std::string, Landmark>> getNearbyLandmarks(const tf2::Vector3 & point)
  {
    std::vector<std::pair<std::string, Landmark>> nearby_landmarks;
    for (const auto & [landmark_name, landmark] : landmarks_) {
      if (tf2::tf2Distance(point, landmark.pose.getOrigin()) <= max_landmark_remove_dist_)
        nearby_landmarks.push_back(make_pair(landmark_name, landmark));
    }
    std::sort(
      begin(nearby_landmarks), end(nearby_landmarks),
      [point](
        const std::pair<std::string, Landmark> & lhs,
        const std::pair<std::string, Landmark> & rhs) {
        return tf2::tf2Distance2(point, lhs.second.pose.getOrigin()) <
               tf2::tf2Distance2(point, rhs.second.pose.getOrigin());
      });
    return nearby_landmarks;
  }

  void changeLandmarkIdCB(const ruvu_mcl_msgs::LandmarkEntryConstPtr & msg)
  {
    tf2::Vector3 point;
    tf2::fromMsg(msg->pose.pose.position, point);

    auto nearby_landmarks = getNearbyLandmarks(point);
    if (nearby_landmarks.size() == 0) return;
    landmarks_.at(nearby_landmarks[0].first).id = msg->id;
    ROS_INFO_STREAM(
      "Changed id of " << nearby_landmarks[0].first << " from " << nearby_landmarks[0].second.id
                       << " to " << msg->id);
    visualization_msgs::InteractiveMarker int_marker;
    interactive_marker_server_.get(nearby_landmarks[0].first, int_marker);
    int_marker.description =
      nearby_landmarks[0].first + ", id: " + (msg->id ? std::to_string(msg->id) : "<none>");
    interactive_marker_server_.insert(int_marker, boost::bind(&Node::markerCB, this, _1));
    interactive_marker_server_.applyChanges();
    saveLandmarks();
    publishLandmarks(ros::Time::now());
  }

  void removeLandmarkCB(const geometry_msgs::PointStampedConstPtr & msg)
  {
    tf2::Vector3 point;
    tf2::fromMsg(msg->point, point);
    auto nearby_landmarks = getNearbyLandmarks(point);
    if (nearby_landmarks.size() == 0) return;
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
      tf2::toMsg(landmark.pose, entry.pose.pose);
      msg.landmarks.push_back(std::move(entry));
    }
    msg.header.frame_id = frame_id_;
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
    tf2::Transform pose;
    tf2::fromMsg(feedback->pose, pose);
    landmarks_.at(feedback->marker_name) = pose;
    interactive_marker_server_.applyChanges();
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP) {
      ROS_INFO_STREAM(
        feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                              << feedback->pose.position.y << ", " << feedback->pose.position.z);
      saveLandmarks();
      publishLandmarks(feedback->header.stamp);
    }
  }

  ros::ServiceServer add_landmark_server_;
  ros::Subscriber change_landmark_id_sub_;
  ros::Subscriber remove_landmark_sub_;
  ros::Publisher landmarks_pub_;
  double max_landmark_remove_dist_;
  uint next_landmark_number_;

  interactive_markers::InteractiveMarkerServer interactive_marker_server_;
  std::map<std::string, Landmark> landmarks_;
  std::filesystem::path path_;

  std::string frame_id_;
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
