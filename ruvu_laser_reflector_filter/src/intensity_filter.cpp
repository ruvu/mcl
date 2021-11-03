// Copyright 2021 RUVU Robotics B.V.

#include <math.h>

#include <utility>

#include "dynamic_reconfigure/server.h"
#include "euclidean_clustering.hpp"
#include "ros/node_handle.h"
#include "ruvu_laser_reflector_filter/ReflectorFilterConfig.h"
#include "ruvu_mcl_msgs/LandmarkEntry.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

std::vector<euclidean_clustering::Point> pointsFromLandmarkList(
  const ruvu_mcl_msgs::LandmarkList & landmarks)
{
  std::vector<euclidean_clustering::Point> points;
  points.resize(landmarks.landmarks.size());
  for (int i = 0; i < landmarks.landmarks.size(); i++) {
    points[i] = euclidean_clustering::Point(
      landmarks.landmarks[i].pose.pose.position.x, landmarks.landmarks[i].pose.pose.position.y);
  }
  return points;
}

ruvu_mcl_msgs::LandmarkList landmarkListFromPoints(
  const std::vector<euclidean_clustering::Point> & points)
{
  ruvu_mcl_msgs::LandmarkList landmarks;
  landmarks.landmarks.resize(points.size());
  for (int i = 0; i < points.size(); i++) {
    landmarks.landmarks[i].pose.pose.position.x = points[i].x;
    landmarks.landmarks[i].pose.pose.position.y = points[i].y;
  }
  return landmarks;
}

class Filter
{
public:
  Filter(ros::NodeHandle nh, ros::NodeHandle private_nh)
  {
    landmarks_pub_ = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmarks", 1, true);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("landmark_markers", 1);
    scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &Filter::scan_cb, this);
  }

  void configure(const ruvu_laser_reflector_filter::ReflectorFilterConfig & config)
  {
    threshold_floor_ = config.threshold_floor;
    threshold_decay_ = config.threshold_decay;
    threshold_multiplier_ = config.threshold_multiplier;
    marker_diameter_ = config.marker_diameter;
    clustering_max_gap_size_ = config.clustering_max_gap_size;
    clustering_max_cluster_size_ = config.clustering_max_cluster_size;
  }

  visualization_msgs::Marker create_marker(
    ruvu_mcl_msgs::LandmarkList landmark_list, const std::string & ns)
  {
    visualization_msgs::Marker m;
    m.header = landmark_list.header;
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::MODIFY;
    m.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
    m.scale.x = marker_diameter_;
    m.scale.y = marker_diameter_;
    m.scale.z = marker_diameter_;
    m.points.reserve(landmark_list.landmarks.size());
    m.colors.reserve(landmark_list.landmarks.size());
    m.lifetime = ros::Duration(0.1);

    std_msgs::ColorRGBA c;
    c.a = 1;
    c.r = 1;
    for (const auto & landmark : landmark_list.landmarks) {
      m.points.push_back(landmark.pose.pose.position);
      m.colors.push_back(c);
    }
    return m;
  }

private:
  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan)
  {
    assert(scan->ranges.size() == scan->intensities.size());

    ruvu_mcl_msgs::LandmarkList landmark_list;
    landmark_list.header = scan->header;

    int cluster_start_idx = -1;
    bool previous = false;
    std::vector<euclidean_clustering::Point> clusters;
    for (size_t i = 0; i < scan->ranges.size(); i++) {
      // Check if intensity is worth passing through the filter
      // This comparison looks ugly, but handles NaN ranges correctly :)
      bool invalid = !(scan->range_min <= scan->ranges[i] && scan->ranges[i] <= scan->range_max);
      bool pass_through = scan->intensities[i] >= threshold_function(scan->ranges[i]) && !invalid;

      if (pass_through) {
        // This if serves to find (initial) cluster suggestions
        if (!previous) {
          cluster_start_idx = i;
          previous = true;
        }

        ruvu_mcl_msgs::LandmarkEntry landmark;
        double angle = scan->angle_min + i * scan->angle_increment;
        landmark.pose.pose.position.x = scan->ranges[i] * tf2Cos(angle);
        landmark.pose.pose.position.y = scan->ranges[i] * tf2Sin(angle);
        landmark_list.landmarks.push_back(std::move(landmark));
        continue;
      }
      if (previous && (!pass_through || i == scan->ranges.size() - 1)) {
        previous = false;
        int median_idx = cluster_start_idx + (i - cluster_start_idx) / 2;
        double angle = scan->angle_min + median_idx * scan->angle_increment;
        double range = scan->ranges[median_idx];
        clusters.push_back(euclidean_clustering::fromPolar(range, angle));
        cluster_start_idx = -1;
      }
    }

    std::vector<euclidean_clustering::Point> points = pointsFromLandmarkList(landmark_list);

    if (clustering_) {
      euclidean_clustering::greedyDistanceClustering(
        points, clusters, clustering_max_gap_size_, clustering_max_cluster_size_);
    }

    landmarks_pub_.publish(landmark_list);
    if (marker_pub_.getNumSubscribers() > 0 && landmark_list.landmarks.size()) {
      auto landmarks_marker = create_marker(landmark_list, "reflector_points");
      ruvu_mcl_msgs::LandmarkList clusters_ll = landmarkListFromPoints(clusters);
      clusters_ll.header = landmark_list.header;
      auto clusters_marker = create_marker(clusters_ll, "reflector_clusters");
      marker_pub_.publish(landmarks_marker);
      marker_pub_.publish(clusters_marker);
    }
  }

  double threshold_function(const double range)
  {
    return threshold_multiplier_ * exp(-threshold_decay_ * range) + threshold_floor_;
  }

  int threshold_multiplier_;
  double threshold_decay_;
  int threshold_floor_;
  double marker_diameter_;
  bool clustering_;
  double clustering_max_gap_size_;
  double clustering_max_cluster_size_;
  ros::Subscriber scan_sub_;
  ros::Publisher landmarks_pub_;
  ros::Publisher marker_pub_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "intensity_filter");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  Filter filter(nh, private_nh);
  dynamic_reconfigure::Server<ruvu_laser_reflector_filter::ReflectorFilterConfig> server;
  server.setCallback([&filter](
                       const ruvu_laser_reflector_filter::ReflectorFilterConfig & config,
                       const uint32_t level) { filter.configure(config); });

  ros::spin();
  return EXIT_SUCCESS;
}
