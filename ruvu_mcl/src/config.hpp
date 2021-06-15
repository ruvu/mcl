// Copyright 2021 RUVU Robotics B.V.

#pragma once

#include <variant>

#include "ruvu_mcl/AMCLConfig.h"

struct BeamModelConfig
{
  double z_hit;
  double z_short;
  double z_max;
  double z_rand;
  double sigma_hit;
  double lambda_short;
  size_t max_beams;
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

  std::variant<BeamModelConfig> laser;
  std::variant<DifferentialMotionModelConfig> model;
};
