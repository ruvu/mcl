// Copyright 2021 RUVU Robotics B.V.

/**
 * This file can be used to quickly benchmark different versions of the gaussian_landmark_model.
 * Prepares the particles and the sensor model and runs one sensor update. If you have a different
 * version of the algorithm, you can add a `BENCHMARK` section below to compare against it.
 */

#include <memory>

#include "../src/config.hpp"
#include "../src/particle_filter.hpp"
#include "../src/rng.hpp"
#include "../src/sensor_models/gaussian_landmark_model.hpp"
#include "../src/sensor_models/landmark.hpp"
#include "benchmark/benchmark.h"

double prob(double a, double var) { return exp(-a * a / 2 / var); }

LandmarkList random_landmarks(const std::shared_ptr<Rng> & rng, int nlandmarks)
{
  LandmarkList map;
  auto dx = rng->uniform_distribution(-100, 100);
  auto dt = rng->uniform_distribution(0, M_2_PI);
  for (int i = 0; i < nlandmarks; i++) {
    tf2::Quaternion q;
    q.setRPY(0, 0, dt());
    map.landmarks.emplace_back(tf2::Transform{q, tf2::Vector3{dx(), dx(), 0}});
  }
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  map.landmarks.emplace_back(tf2::Transform{q, tf2::Vector3{2.1, 2, 0}});
  map.landmarks.emplace_back(tf2::Transform{q, tf2::Vector3{2.1, 2.1, 0}});
  return map;
}

ParticleFilter random_particles(const std::shared_ptr<Rng> & rng, int nparticles)
{
  ParticleFilter pf;

  auto dx = rng->normal_distribution(0, 1);
  auto dt = rng->uniform_distribution(0, 1);
  for (int i = 0; i < nparticles; i++) {
    tf2::Quaternion q;
    q.setRPY(0, 0, dt());
    pf.particles.emplace_back(tf2::Transform{q, tf2::Vector3{dx(), dx(), 0}}, 1. / nparticles);
  }
  return pf;
}

void gaussian_landmark_model(benchmark::State & state)  // NOLINT.
{
  auto nlandmarks = 1000;
  auto nparticles = 5000;

  auto rng = std::make_shared<Rng>(123);

  // create a random map
  auto map = random_landmarks(rng, nlandmarks);

  // create a random measurement
  LandmarkList data;
  tf2::Quaternion q;
  data.landmarks.emplace_back(tf2::Transform{q, tf2::Vector3{2, 2, 0}});

  // create random particles
  auto pf = random_particles(rng, nparticles);

  // create the model
  GaussianLandmarkModelConfig config;
  config.z_rand = 0.5;
  config.landmark_sigma_r = 0.1;
  config.landmark_sigma_t = 0.1;
  GaussianLandmarkModel model{config, map};

  for (auto _ : state) {
    model.sensor_update(&pf, data);
  }

  benchmark::DoNotOptimize(pf);
}
BENCHMARK(gaussian_landmark_model)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
