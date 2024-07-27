// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_CLAMP_H
#define LUM_COMMON_UTILS_CLAMP_H

#include <cassert>
#include <functional>

namespace lum {
namespace common {
namespace utils {

template <class T, class Compare>
constexpr const T& clamp(const T& v, const T& lo, const T& hi, Compare comp)
{
  // https://reviews.llvm.org/D31130
  // NOLINTNEXTLINE(hicpp-no-array-decay,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  assert(!comp(hi, lo));
  return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
}

template <class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
  return clamp(v, lo, hi, std::less<>{});
}

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_CLAMP_H
