// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include <tuple>

namespace ruvu_mcl
{
using Key = std::tuple<int, int, int>;

/**
 * @brief Hash function for Keys
 *
 * This class can be used as hash function in unordered_map<Key> or unordered_set<Key>.
 */
struct KeyHasher
{
  std::size_t operator()(const Key & key) const noexcept;
};
}  // namespace ruvu_mcl
