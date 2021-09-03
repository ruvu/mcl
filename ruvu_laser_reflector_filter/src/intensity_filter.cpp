// Copyright 2021 RUVU Robotics B.V.

#include <math.h>

#include <utility>

#include "dynamic_reconfigure/server.h"
#include "ros/node_handle.h"
#include "ruvu_laser_reflector_filter/ReflectorFilterConfig.h"
#include "ruvu_mcl_msgs/LandmarkEntry.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Scalar.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

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
    publish_markers_ = config.publish_markers;
    marker_diameter_ = config.marker_diameter;
  }

  void publish_markers(ruvu_mcl_msgs::LandmarkList landmark_list)
  {
    visualization_msgs::Marker m;
    m.header = landmark_list.header;
    m.ns = "measured_landmarks";
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
    marker_pub_.publish(std::move(m));
  }

private:
  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan)
  {
    assert(scan->ranges.size() == scan->intensities.size());

    ruvu_mcl_msgs::LandmarkList landmark_list;
    landmark_list.header = scan->header;

    for (size_t i = 0; i < scan->ranges.size(); i++) {
      if (scan->intensities[i] >= threshold_function(scan->ranges[i])) {
        ruvu_mcl_msgs::LandmarkEntry landmark;
        double angle = scan->angle_min + i * scan->angle_increment;
        landmark.pose.pose.position.x = scan->ranges[i] * tf2Cos(angle);
        landmark.pose.pose.position.y = scan->ranges[i] * tf2Sin(angle);
        landmark_list.landmarks.push_back(std::move(landmark));
      }
    }
    if (landmark_list.landmarks.size() > 0) {
      landmarks_pub_.publish(landmark_list);
      if (publish_markers_) publish_markers(landmark_list);
    }
  }

  double threshold_function(const double range)
  {
    return threshold_multiplier_ * exp(-threshold_decay_ * range) + threshold_floor_;
  }

  int threshold_multiplier_;
  double threshold_decay_;
  int threshold_floor_;
  bool publish_markers_;
  double marker_diameter_;
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
