// Copyright 2021 RUVU Robotics B.V.

#include <gnuplot-iostream.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "../src/particle_filter.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "rosbag/bag.h"
#include "rosbag/query.h"
#include "rosbag/view.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    puts("usage: plot_adaptive BAGFILE");
    exit(EXIT_FAILURE);
  }

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  std::vector<std::string> topics = {"/mcl/cloud", "/mcl/pose"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::vector<std::pair<double, double>> cov_data;
  std::vector<std::pair<double, uint32_t>> nr_particles;
  std::cout << "Start processing bag\n";
  for (const auto & msg : view) {
    if (visualization_msgs::MarkerConstPtr cloud = msg.instantiate<visualization_msgs::Marker>()) {
      nr_particles.emplace_back(cloud->header.stamp.toSec(), cloud->colors.size() / 2);
    } else if (
      geometry_msgs::PoseWithCovarianceStampedConstPtr pose =
        msg.instantiate<geometry_msgs::PoseWithCovarianceStamped>()) {
      double cov_size = std::fabs(pose->pose.covariance[0]) + std::fabs(pose->pose.covariance[7]) +
                        std::fabs(pose->pose.covariance[35]);
      cov_data.emplace_back(pose->header.stamp.toSec(), cov_size);
    } else {
      std::cout << "Unknown message\n";
    }
  }

  if (cov_data.empty()) std::cout << "No covariance data found!\n";
  if (nr_particles.empty()) std::cout << "No filter size data found\n";

  Gnuplot gp;
  gp << "set multiplot layout 2,1 rowsfirst\n";
  gp << "set autoscale\n";
  gp << "plot" << gp.file1d(cov_data) << "title 'Covariance size'\n";
  gp << "plot" << gp.file1d(nr_particles) << "title 'Nr particles'\n";

  bag.close();
  std::cout << "Done\n";
  return EXIT_SUCCESS;
}
