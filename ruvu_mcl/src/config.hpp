// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <string>
#include <variant>

// forward declare
namespace ruvu_mcl
{
class AMCLConfig;
}

struct BeamModelConfig
{
  double z_hit;
  double z_short;
  double z_max;
  double z_rand;
  double sigma_hit;
  double lambda_short;
  size_t max_beams;
  std::string global_frame_id;  // for visualization
};

struct LikelihoodFieldModelConfig
{
  double z_hit;
  double z_rand;
  double sigma_hit;
  size_t max_beams;
  std::string global_frame_id;  // for visualization
};

struct DifferentialMotionModelConfig
{
  double alpha1;
  double alpha2;
  double alpha3;
  double alpha4;
};

struct Config
{
  Config(const ruvu_mcl::AMCLConfig & config);
  Config() = default;

  size_t max_particles;
  double update_min_d;
  double update_min_a;
  int resample_interval;
  bool selective_resampling;
  double transform_tolerance;

  std::variant<BeamModelConfig, LikelihoodFieldModelConfig> laser;
  std::variant<DifferentialMotionModelConfig> model;

  std::string odom_frame_id;
  std::string base_frame_id;
  std::string global_frame_id;
};
