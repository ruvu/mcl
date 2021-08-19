// Copyright 2021 RUVU Robotics B.V.

#include <utility>

#include "dynamic_reconfigure/server.h"
#include "ros/node_handle.h"
#include "ruvu_laser_reflector_filter/ReflectorFilterConfig.h"
#include "ruvu_mcl_msgs/LandmarkEntry.h"
#include "ruvu_mcl_msgs/LandmarkList.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Scalar.h"

class Filter
{
public:
  Filter(ros::NodeHandle nh, ros::NodeHandle private_nh)
  {
    landmarks_pub_ = nh.advertise<ruvu_mcl_msgs::LandmarkList>("landmarks", 1, true);
    scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &Filter::scan_cb, this);
  }

  void configure(const ruvu_laser_reflector_filter::ReflectorFilterConfig & config)
  {
    intensity_threshold_ = config.intensity_threshold;
  }

private:
  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan)
  {
    assert(scan->ranges.size() == scan->intensities.size());

    ruvu_mcl_msgs::LandmarkList landmark_list;
    landmark_list.header = scan->header;
    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
      if (scan->intensities[i] >= intensity_threshold_)
      {
        ruvu_mcl_msgs::LandmarkEntry landmark;
        double angle = scan->angle_min + i * scan->angle_increment;
        landmark.pose.pose.position.x = scan->ranges[i] * tf2Cos(angle);
        landmark.pose.pose.position.y = scan->ranges[i] * tf2Sin(angle);
        landmark_list.landmarks.push_back(std::move(landmark));
      }
    }
    if (landmark_list.landmarks.size() > 0)
    {
      landmarks_pub_.publish(landmark_list);
    }
  }

  int intensity_threshold_;
  ros::Subscriber scan_sub_;
  ros::Publisher landmarks_pub_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "intensity_filter");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  Filter filter(nh, private_nh);
  dynamic_reconfigure::Server<ruvu_laser_reflector_filter::ReflectorFilterConfig> server;
  server.setCallback(
    [&filter](
      const ruvu_laser_reflector_filter::ReflectorFilterConfig & config, const uint32_t level)
    {
      filter.configure(config);
    });  //NOLINT

  ros::spin();
  return EXIT_SUCCESS;
}
