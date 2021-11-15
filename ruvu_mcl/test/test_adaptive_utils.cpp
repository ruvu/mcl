// Copyright 2021 RUVU Robotics B.V.

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "../src/adaptive/utils.hpp"
#include "ros/console.h"

using ruvu_mcl::KeyHasher;

/**
 * @brief Check if all elements are unique
 */
template <class It>
bool is_all_unique(It begin, It end)
{
  using value_type = typename std::iterator_traits<It>::value_type;

  std::vector<value_type> numbers{begin, end};  // copy
  std::sort(numbers.begin(), numbers.end());
  auto it = std::unique(numbers.begin(), numbers.end());
  return it == numbers.end();
}

TEST(is_all_unique, 123)
{
  size_t numbers[] = {1, 2, 3};
  ASSERT_TRUE(is_all_unique(std::begin(numbers), std::end(numbers)));
}

TEST(is_all_unique, 011)
{
  size_t numbers[] = {0, 1, 1};
  ASSERT_FALSE(is_all_unique(std::begin(numbers), std::end(numbers)));
}

TEST(AdaptiveUtils, test1)
{
  std::vector<size_t> hashes;
  KeyHasher hasher;

  // calculate hashes for every combination of 0s and 1s
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 2; ++k) {
        hashes.push_back(hasher({i, j, k}));
      }
    }
  }
  assert(hashes.size() == 2 * 2 * 2);

  // verify that each hash is unique
  ASSERT_TRUE(is_all_unique(hashes.begin(), hashes.end()));
}

int main(int argc, char ** argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
