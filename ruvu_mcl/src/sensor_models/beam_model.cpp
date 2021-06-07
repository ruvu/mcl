#include "./beam_model.hpp"

#include "../map.hpp"
#include "ros/console.h"
#include "ros/node_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/Marker.h"

BeamModel::BeamModel(const Parameters & parameters, const std::shared_ptr<const Map> & map)
: parameters_(parameters), map_(map)
{
  ros::NodeHandle nh("~");
  debug_pub_ = nh.advertise<visualization_msgs::Marker>("beam_model", 1);
  // TODO: implement
}

bool BeamModel::sensor_update(ParticleFilter * pf, const LaserData & data)
{
  ROS_INFO("BeamModel::sensor_update");
  auto step = (data.ranges.size() - 1) / (parameters_.max_beams - 1);

  ROS_INFO("step size=%zu", step);

  bool first = true;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::MODIFY;
  tf2::toMsg(tf2::Transform::getIdentity(), marker.pose);
  marker.scale.x = 0.01;
  marker.color.r = 1;
  marker.color.a = 1;

  for (auto & particle : *pf) {
    for (std::size_t i = 0; i < data.ranges.size(); i += step) {
      double obs_range = data.ranges[i];
      auto q = data.get_angle(i);

      if (std::isinf(obs_range)) continue;  // TODO: don't skip infs

      double max_range = 10;  // TODO: make configurable
      double map_range = map_->calc_range(
        particle.pose * data.pose.getOrigin(),
        particle.pose * data.pose * tf2::quatRotate(q, {max_range, 0, 0}));
      // ROS_INFO("obs: %f map: %f", obs_range, map_range);
      if (std::isinf(map_range)) continue;  // TODO: don't skip infs

      if (first) {
        geometry_msgs::Point p1, p2;
        tf2::toMsg(particle.pose * data.pose.getOrigin(), p1);
        tf2::toMsg(
          particle.pose * data.pose * tf2::quatRotate(q, tf2::Vector3{map_range, 0, 0}), p2);
        marker.points.push_back(std::move(p1));
        marker.points.push_back(std::move(p2));
      }
    }

    first = false;
  }

  debug_pub_.publish(std::move(marker));

  return true;
}
