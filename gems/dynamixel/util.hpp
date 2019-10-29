#pragma once

#include <algorithm>
#include <sstream>
#include <string>

template <typename T>
T clamp(const T& value, const T& low, const T& high) {
  return std::max(std::min(value, high), low);
}
