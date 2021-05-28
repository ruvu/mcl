#include <message_filters/subscriber.h>
#include <pf/rv_samp.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <cstdio>
#include <iostream>

using Input = Eigen::Matrix<double, 2, 1>;

static constexpr double g = 9.81;

class Model
{
public:
  double height = 0;
  double velocity = 0;

  void update(double dt)
  {
    velocity += -g * dt;
    height += velocity * dt;
  }

private:
};

class Particle
{
public:
  double height;
  double velocity;
  double weight = 1;

  explicit Particle(double height, double velocity) : height(height), velocity(velocity) {}
  void update(double dt)
  {
    velocity += -g * dt;
    height += velocity * dt;
  }
};

class SensorModel
{
public:
  void update(std::vector<Particle> & particles, double data)
  {
    for (auto & particle : particles) {
      auto z = particle.height - data;
      auto p = exp(-z * z / 2 / sigma / sigma);
      particle.weight *= p;
    }
  }

private:
  const double sigma = 1;
};

class Gps
{
public:
  Gps(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : laser_scan_sub_(nh, "scan", 100),
    laser_scan_filter_(laser_scan_sub_, tf_buffer, "odom", 100, nh)
  {
    laser_scan_filter_.registerCallback(&Gps::scan_cb, this);

    for (int i = 0; i < 10; ++i) {
      particles_.emplace_back(distribution.sample(), distribution.sample());
    }
  }

private:
  void scan_cb(const sensor_msgs::LaserScan::ConstPtr & scan)
  {
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header.stamp = scan->header.stamp;
    odom_pose.header.frame_id = "base_link";
    tf2::toMsg(tf2::Transform::getIdentity(), odom_pose.pose);
    ROS_INFO("lookup at time %f", odom_pose.header.stamp.toSec());
    try {
      tf_buffer.transform(odom_pose, odom_pose, "odom");
    } catch (const tf2::ExtrapolationException & e) {
      ROS_WARN("laser scan transform failed: (%s)", e.what());
    }

    ROS_INFO("%f %f", odom_pose.pose.position.x, odom_pose.pose.position.y);
  }

  void update(const ros::Time & dt)
  {
    model_.update(0.1);

    for (auto & particle : particles_) {
      particle.update(0.1);
    }

    double measurement = distribution.sample() * model_.height;
    sensor_model_.update(particles_, measurement);
  }

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{tf_buffer};
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_scan_filter_;

  pf::rvsamp::UnivNormSampler<double> distribution{0, 0.1};
  std::vector<Particle> particles_;
  Model model_ = {};
  SensorModel sensor_model_ = {};
  ros::Subscriber scan_sub_;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener{tf_buffer};
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan> laser_scan_filter_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "gps");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_INFO("hello world");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh{"~"};
  Gps gps{nh, private_nh};
  ros::spin();
  return EXIT_SUCCESS;
}
