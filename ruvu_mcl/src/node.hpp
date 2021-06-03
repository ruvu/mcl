#include "./map.hpp"
#include "./motion_models/differential_motion_model.hpp"
#include "./particle_filter.hpp"
#include "./sensor_models/beam_model.hpp"
#include "message_filters/subscriber.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pf/rv_samp.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/Marker.h"

class Node
{
public:
  Node(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  void scan_cb(const sensor_msgs::LaserScan::ConstPtr & scan);
  void map_cb(const nav_msgs::OccupancyGrid::ConstPtr & map);
  tf2::Transform get_odom_pose(const ros::Time & time);
  void publish_particle_cloud(const ros::Time & time);

  // data input
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{tf_buffer};
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_scan_filter_;
  ros::Subscriber map_sub_;

  // data output
  ros::Publisher cloud_pub_;

  // internals
  tf2::Transform last_odom_pose_ = {};
  pf::rvsamp::UnivNormSampler<double> distribution{0, 0.1};
  ParticleFilter particles_;
  DifferentialMotionModel model_ = {0.1, 0.1, 0.1, 0.1};
  Map map_;
  std::unique_ptr<Laser> laser_;
};
