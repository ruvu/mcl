#include <fstream>
#include <nlohmann/json.hpp>

#include "./filesystem.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "ros/node_handle.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

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
  Node(ros::NodeHandle nh, ros::NodeHandle private_nh) : landmarks_(), path_()
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
      landmarks_ = j.get<std::vector<Landmark>>();
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

    landmarks_.emplace_back(tf2::Transform{orientation, position});

    // save
    {
      json j = landmarks_;
      std::ofstream file(path_);
      file << std::setw(2) << j;
    }

    publish_landmarks(msg->header.stamp);
  }

  void publish_landmarks(const ros::Time & stamp)
  {
    ruvu_mcl_msgs::LandmarkList msg;
    for (const auto & landmark : landmarks_) {
      ruvu_mcl_msgs::LandmarkEntry entry;
      tf2::toMsg(landmark.pose, entry.pose.pose);
      msg.landmarks.push_back(std::move(entry));
    }
    msg.header.frame_id = "map";
    msg.header.stamp = stamp;
    landmarks_pub_.publish(msg);
  }

  ros::Subscriber pose_sub_;
  ros::Publisher landmarks_pub_;

  std::vector<Landmark> landmarks_;
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
