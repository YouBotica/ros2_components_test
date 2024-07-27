// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_INTERNAL_UTILS_H
#define LUM_COMMON_TYPES_INTERNAL_UTILS_H

#include <iomanip>
#include <sstream>
#include <string>

#include <lum_common_types/common.h>

namespace lum {
namespace common {
namespace types {
namespace utils {

/// @brief Compile time function to perform explicit casting of enum member to
/// underlying type
/// @param [in] enumerator arbitrary enumerator queried
/// @return compile time constant value of enum member
template <typename E>
constexpr std::underlying_type_t<E> toUnderlyingType(const E& enumerator) noexcept
{
  return static_cast<std::underlying_type_t<E>>(enumerator);
}

/// @brief Utility function to get a string for any given timestamp
/// @param [in] timestamp lum::common::types::Time
/// @param [in] scale_factor This factor converts nanoseconds to another unit
/// (s/ms,etc). Eg: to get output in seconds the factor should be 1e-9, to convert to milliseconds
/// pass 1e-6, to convert to microseconds pass 1e-3, to convert to nanoseconds pass 1e1.
/// @return String representing time in seconds
inline std::string timeToString(const types::Time& timestamp, double scale_factor = 1.0)
{
  std::stringstream time_string{};
  time_string << std::setprecision(20) << static_cast<double>(timestamp.count()) * scale_factor
              << std::fixed;

  return time_string.str();
}

/// @brief Get absolute value of time difference between two time stamps
/// @param [in] current_time const reference to time stamp for current frame
/// @param [in] prev_time const reference to time stamp for previous frame
/// @return Absolute difference in seconds as double
inline double timeDifference(const types::Time& current_time, const types::Time& prev_time)
{
  const auto time_difference = current_time - prev_time;
  return std::abs(toSeconds(time_difference));
}

} // namespace utils
} // namespace types
} // namespace common
} // namespace lum

#endif
