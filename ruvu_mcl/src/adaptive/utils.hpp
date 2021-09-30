// Copyright 2021 RUVU Robotics B.V.
#pragma once

#include <tuple>

using Key = std::tuple<int, int, int>;

struct KeyHasher
{
  std::size_t operator()(const Key & key) const noexcept;
};
