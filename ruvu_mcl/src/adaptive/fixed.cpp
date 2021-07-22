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

void Fixed::before_odometry_update(ParticleFilter * pf)
{
  auto & particles = pf->particles;

  // delete particles if we have too much
  if (particles.size() > max_particles_) {
    ROS_INFO("deleting particles");
    particles.erase(particles.begin() + max_particles_, particles.end());
    assert(particles.size() == max_particles_);
  }

  // copy particles if we have too little
  if (particles.size() < max_particles_) ROS_INFO("duplicating particles");
  auto original_size = particles.size();
  decltype(original_size) i = 0;
  while (particles.size() < max_particles_) {
    particles.push_back(particles[i++ % original_size]);
  }
}
