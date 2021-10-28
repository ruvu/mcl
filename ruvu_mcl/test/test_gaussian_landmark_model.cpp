// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include <memory>

#include "../src/config.hpp"
#include "../src/particle_filter.hpp"
#include "../src/sensor_models/gaussian_landmark_model.hpp"
#include "ros/console.h"

class GaussianLandmarkModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config.z_rand = 0.1;
    config.landmark_sigma_r = 0.1;
    config.landmark_sigma_t = 0.1;

    model.reset();

    pf.particles.clear();
  }

  void SetUp(const LandmarkList & map)
  {
    model = std::make_unique<GaussianLandmarkModel>(config, map);
  }

  GaussianLandmarkModelConfig config;
  std::unique_ptr<GaussianLandmarkModel> model;
  ParticleFilter pf;
};

TEST_F(GaussianLandmarkModelTest, test_empty)
{
  LandmarkList map;
  SetUp(map);
  pf.particles.emplace_back(tf2::Transform::getIdentity(), 1);

  LandmarkList data;
  model->sensor_update(&pf, data);
  ASSERT_DOUBLE_EQ(pf.particles.front().weight, 1);
}

TEST_F(GaussianLandmarkModelTest, test_range)
{
  LandmarkList map;
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI);
  map.landmarks.emplace_back(tf2::Transform{q, tf2::Vector3{1, 0, 0}});
  SetUp(map);
  pf.particles.emplace_back(tf2::Transform::getIdentity(), 1);
  pf.particles.emplace_back(
    tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{config.landmark_sigma_r, 0, 0}}, 1);
  pf.particles.emplace_back(
    tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{-10, 0, 0}}, 1);

  LandmarkList data;
  data.landmarks.emplace_back(
    tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{1, 0, 0}});
  model->sensor_update(&pf, data);
  ASSERT_NEAR(pf.particles[0].weight, 0.572, 0.01);  // Perfect hit
  ASSERT_NEAR(pf.particles[1].weight, 0.367, 0.01);  // 1x sigma
  ASSERT_NEAR(pf.particles[2].weight, 0.057, 0.01);  // Random particle
}

TEST_F(GaussianLandmarkModelTest, test_angle)
{
  LandmarkList map;
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI);
  map.landmarks.emplace_back(tf2::Transform{q, tf2::Vector3{1, 0, 0}});
  SetUp(map);
  pf.particles.emplace_back(tf2::Transform::getIdentity(), 1);
  q.setRPY(0, 0, config.landmark_sigma_t);
  pf.particles.emplace_back(tf2::Transform{q, tf2::Vector3{0, 0, 0}}, 1);
  q.setRPY(0, 0, M_PI);
  pf.particles.emplace_back(tf2::Transform{q, tf2::Vector3{0, 0, 0}}, 1);

  LandmarkList data;
  data.landmarks.emplace_back(
    tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{1, 0, 0}});
  model->sensor_update(&pf, data);
  ASSERT_NEAR(pf.particles[0].weight, 0.572, 0.01);  // Perfect hit
  ASSERT_NEAR(pf.particles[1].weight, 0.367, 0.01);  // 1x sigma
  ASSERT_NEAR(pf.particles[2].weight, 0.057, 0.01);  // Random particle
}

/**
 * @brief Test measurement with a laser with an offset and a particle with an offset
 */
TEST_F(GaussianLandmarkModelTest, test_laser_and_particle_offset)
{
  LandmarkList map;
  map.landmarks.emplace_back(tf2::Transform{tf2::Quaternion::getIdentity(), tf2::Vector3{1, 5, 0}});
  SetUp(map);
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  pf.particles.emplace_back(tf2::Transform{q, tf2::Vector3{2, 1, 0}}, 1);
  pf.particles.emplace_back(tf2::Transform::getIdentity(), 1);  // to counter normalization

  LandmarkList data;
  data.landmarks.emplace_back(tf2::Transform{q, tf2::Vector3{0, 2 * M_SQRT2, 0}});
  q.setRPY(0, 0, -M_PI / 4);
  data.pose = tf2::Transform{q, tf2::Vector3{2, -1, 0}};
  model->sensor_update(&pf, data);
  ROS_INFO("pf: %f %f", pf.particles[0].weight, pf.particles[1].weight);
  ASSERT_DOUBLE_EQ(pf.particles[0].weight, 1.0 / (1 + config.z_rand));
  ASSERT_DOUBLE_EQ(pf.particles[1].weight, 0.1 / (1 + config.z_rand));
}

/**
 * @brief Test measurement with a laser offset and a particle to the left
 */

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
