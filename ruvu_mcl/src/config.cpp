// Copyright 2021 RUVU Robotics B.V.

#include "./config.hpp"

#include <numeric>

#include "ruvu_mcl/AMCLConfig.h"

void normalize(std::initializer_list<double *> zs)
{
  double z_total =
    std::accumulate(zs.begin(), zs.end(), 0.0, [](double acc, double * z) { return acc + *z; });
  for (double * z : zs) *z /= z_total;
}

Config::Config(const ruvu_mcl::AMCLConfig & config)
{
  min_particles = config.min_particles;
  max_particles = config.max_particles;
  update_min_d = config.update_min_d;
  update_min_a = config.update_min_a;
  resample_interval = config.resample_interval;
  selective_resampling = config.selective_resampling;
  transform_tolerance = config.transform_tolerance;

  tf2::Quaternion q;
  q.setRPY(0, 0, config.initial_pose_a);
  initial_pose = tf2::Transform{q, tf2::Vector3{config.initial_pose_x, config.initial_pose_y, 0}};
  initial_cov = {0};
  initial_cov[0 * 6 + 0] = config.initial_cov_xx;
  initial_cov[1 * 6 + 1] = config.initial_cov_yy;
  initial_cov[5 * 6 + 5] = config.initial_cov_aa;

  if (config.odom_model_type == ruvu_mcl::AMCL_diff_const) {
    DifferentialMotionModelConfig m;
    m.alpha1 = config.odom_alpha1;
    m.alpha2 = config.odom_alpha2;
    m.alpha3 = config.odom_alpha3;
    m.alpha4 = config.odom_alpha4;
    model = m;
  } else {
    std::ostringstream ss;
    ss << "motion model " << config.odom_model_type << " is not yet implemented";
    throw std::runtime_error(ss.str());
  }

  if (config.laser_model_type == ruvu_mcl::AMCL_beam_const) {
    BeamModelConfig c;
    c.z_hit = config.laser_z_hit;
    c.z_short = config.laser_z_short;
    c.z_max = config.laser_z_max;
    c.z_rand = config.laser_z_rand;
    c.sigma_hit = config.laser_sigma_hit;
    c.lambda_short = config.laser_lambda_short;
    c.max_beams = config.laser_max_beams;
    c.global_frame_id = config.global_frame_id;
    normalize({&c.z_hit, &c.z_short, &c.z_max, &c.z_rand});
    laser = c;
  } else if (config.laser_model_type == ruvu_mcl::AMCL_likelihood_field_const) {
    LikelihoodFieldModelConfig c;
    c.z_hit = config.laser_z_hit;
    c.z_rand = config.laser_z_rand;
    c.sigma_hit = config.laser_sigma_hit;
    c.max_beams = config.laser_max_beams;
    c.global_frame_id = config.global_frame_id;
    normalize({&c.z_hit, &c.z_rand});
    laser = c;
  } else {
    std::ostringstream ss;
    ss << "sensor model " << config.laser_model_type << " is not yet implemented";
    throw std::runtime_error(ss.str());
  }

  if (config.landmark_model_type == ruvu_mcl::AMCL_landmark_gaussian_const) {
    GaussianLandmarkModelConfig c;
    c.z_rand = config.landmark_z_rand;
    c.landmark_sigma_r = config.landmark_sigma_r;
    c.landmark_sigma_t = config.landmark_sigma_t;
    c.landmark_max_r_confidence = config.landmark_max_r_confidence;
    c.global_frame_id = config.global_frame_id;
    landmark = c;
  } else if (config.landmark_model_type == ruvu_mcl::AMCL_landmark_likelihood_field_const) {
    LandmarkLikelihoodFieldModelConfig c;
    c.z_hit = config.landmark_z_hit;
    c.z_rand = config.landmark_z_rand;
    c.sigma_hit = config.landmark_sigma_hit;
    c.global_frame_id = config.global_frame_id;
    normalize({&c.z_hit, &c.z_rand});
    landmark = c;
  } else {
    std::ostringstream ss;
    ss << "sensor model " << config.laser_model_type << " is not yet implemented";
    throw std::runtime_error(ss.str());
  }

  if (config.adaptive_type == ruvu_mcl::AMCL_kld_sampling) {
    KLDSamplingConfig c;
    c.min_particles = config.min_particles;
    c.max_particles = config.max_particles;
    c.kld_err = config.kld_err;
    c.kld_z = config.kld_z;
    c.xy_grid_size = config.xy_grid_size;
    c.theta_grid_size = config.theta_grid_size;
    adaptive = c;
  } else if (config.adaptive_type == ruvu_mcl::AMCL_split_and_merge) {
    SplitAndMergeConfig c;
    c.xy_grid_size = config.xy_grid_size;
    c.theta_grid_size = config.theta_grid_size;
    c.split_weight = config.split_weight;
    adaptive = c;
  }

  odom_frame_id = config.odom_frame_id;
  base_frame_id = config.base_frame_id;
  global_frame_id = config.global_frame_id;
}
