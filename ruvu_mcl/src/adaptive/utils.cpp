// Copyright 2021 RUVU Robotics B.V.

#include "./utils.hpp"

#include <boost/functional/hash.hpp>

namespace ruvu_mcl
{
std::size_t KeyHasher::operator()(const Key & key) const noexcept
{
  std::size_t seed = 0;
  boost::hash_combine(seed, key);
  return seed;
}
}  // namespace ruvu_mcl
