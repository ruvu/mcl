// Copyright 2021 RUVU Robotics B.V.

#include "./fixed.hpp"

#include "../config.hpp"
#include "../particle_filter.hpp"
#include "ros/console.h"

constexpr auto name = "fixed";

Fixed::Fixed(const Config & config) : max_particles_(config.max_particles)
{
  ROS_INFO_NAMED(name, "using a fixed number of particles");
}

int Fixed::calc_needed_particles(const ParticleFilter & pf) const { return max_particles_; }
