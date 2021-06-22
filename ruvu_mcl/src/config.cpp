// Copyright 2021 RUVU Robotics B.V.

#include "./config.hpp"

Config::Config(const ruvu_mcl::AMCLConfig & config)
{
  max_particles = config.max_particles;
  update_min_d = config.update_min_d;
  update_min_a = config.update_min_a;
  resample_interval = config.resample_interval;
  selective_resampling = config.selective_resampling;

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
    laser = c;
  } else if (config.laser_model_type == ruvu_mcl::AMCL_likelihood_field_const) {
    LikelihoodFieldModelConfig c;
    c.z_hit = config.laser_z_hit;
    c.z_rand = config.laser_z_rand;
    c.sigma_hit = config.laser_sigma_hit;
    c.max_beams = config.laser_max_beams;
    c.global_frame_id = config.global_frame_id;
    laser = c;
  } else {
    std::ostringstream ss;
    ss << "sensor model " << config.laser_model_type << " is not yet implemented";
    throw std::runtime_error(ss.str());
  }

  odom_frame_id = config.odom_frame_id;
  base_frame_id = config.base_frame_id;
  global_frame_id = config.global_frame_id;
}
