// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_COMMON_H
#define LUM_COMMON_TYPES_COMMON_H

#include <chrono>
#include <cmath>
#include <cstdint>

namespace lum {
namespace common {
namespace types {

/// @namespace lum::common::types Common data types

/// @brief Type for representing time at nanosecond resolution
using Time = std::chrono::nanoseconds;

/// @brief Convert duration to seconds (double)
/// @tparam Rep Representation of duration (inferred)
/// @tparam Period Period of duration (inferred)
/// @param[in] duration Duration to convert
/// @return Seconds as double
template <typename Rep, typename Period>
inline double toSeconds(const std::chrono::duration<Rep, Period>& duration) noexcept(true)
{
  using Seconds = std::chrono::duration<double, std::chrono::seconds::period>;
  return Seconds{duration}.count();
}

/// @brief Convert seconds (double) to Time (std::chrono::duration)
/// @param [in] seconds Seconds as double
/// @return Time at nanoseconds resolution
// NOLINTNEXTLINE(hicpp-signed-bitwise) error in GCC cmath header
inline Time toNanoseconds(const double seconds) noexcept((math_errhandling & MATH_ERREXCEPT) == 0)
{
  return Time{static_cast<std::int64_t>(std::trunc(seconds * static_cast<double>(std::nano::den)))};
}

} // namespace types
} // namespace common
} // namespace lum

#endif
