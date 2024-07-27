// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_CONTAINS_H
#define LUM_COMMON_UTILS_CONTAINS_H

#include <algorithm>

namespace lum {
namespace common {
namespace utils {

template <class C, class T>
auto contains(const C& v, const T& x) -> decltype(end(v), true)
{
  return end(v) != std::find(begin(v), end(v), x);
}

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_CONTAINS_H
