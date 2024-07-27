// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MATH_UTILS_H
#define LUM_COMMON_MATH_UTILS_H

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <vector>

/// @def Template parameter restricted to numeric types
#define LUM_NUMERIC_TEMPLATE_TYPE                                                                  \
  template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>

/// @def Template parameter restricted to floating point types
#define LUM_FLOATING_TEMPLATE_TYPE                                                                 \
  template <typename T,                                                                            \
            typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>

namespace lum {
namespace common {
namespace math {
namespace utils {

/// @brief Clamps numeric value between min and max
/// @param[in] x input value
/// @param[in] min minimum value
/// @param[in] max maximum value
/// @return max if x > max, min if x < min, else x
LUM_NUMERIC_TEMPLATE_TYPE
constexpr T clamp(const T x, const T min, const T max) noexcept
{
  return std::min<T>(std::max<T>(x, min), max);
}

/// @brief Normalize numeric value between min-max to between 0-1
/// @param[in] x input value
/// @param[in] min minimum value
/// @param[in] max maximum value
/// @return (max if x > max, min if x < min, else x) / (max - min)
LUM_NUMERIC_TEMPLATE_TYPE
inline T normalize(T x, T min, T max)
{
  const auto clamped = clamp<T>(x, min, max);
  return (clamped - min) / (max - min);
}

/// @brief Converts angle in degrees to angle in radians
/// @param[in] angle_degrees angle in degrees
/// @return angle in radians
LUM_FLOATING_TEMPLATE_TYPE
constexpr T deg2rad(T angle_degrees) noexcept
{
  return angle_degrees * static_cast<T>(M_PI / 180.0);
}

/// @brief Converts angle in radians to angle in degrees
/// @param[in] angle_radians angle in radians
/// @return angle in degrees
LUM_FLOATING_TEMPLATE_TYPE
constexpr T rad2deg(T angle_radians) noexcept
{
  return angle_radians * static_cast<T>(180.0 / M_PI);
}

/// @brief Converts temperature in Kelvin to temperature in Celsius
/// @param[in] temp_kelvin temperature in Kelvin
/// @return temperature in Celsius
LUM_FLOATING_TEMPLATE_TYPE
constexpr T kelvin2celsius(T temp_kelvin) noexcept
{
  return temp_kelvin - static_cast<T>(273.15);
}

/// @brief Converts temperature in Celsius to temperature in Kelvin
/// @param[in] temp_celcius temperature in Celsius
/// @return temperature in Kelvin
LUM_FLOATING_TEMPLATE_TYPE
constexpr T celsius2kelvin(T temp_celcius) noexcept
{
  return temp_celcius + static_cast<T>(273.15);
}

/// @brief Converts temperature in Celsius to temperature in Fahrenheit
/// @param[in] temp_celsius temperature in Celsius
/// @return temperature in Fahrenheit
LUM_FLOATING_TEMPLATE_TYPE
constexpr T celsius2fahrenheit(T temp_celcius) noexcept
{
  return (temp_celcius * static_cast<T>(9.0 / 5.0)) + static_cast<T>(32);
}

/// @brief Converts temperature in Kelvin to temperature in Fahrenheit
/// @param[in] temp_kelvin temperature in Kelvin
/// @return temperature in Fahrenheit
LUM_FLOATING_TEMPLATE_TYPE
constexpr T kelvin2fahrenheit(T temp_kelvin) noexcept
{
  return (temp_kelvin - static_cast<T>(273.15)) * static_cast<T>(9.0 / 5.0) + static_cast<T>(32);
}

/// @brief Converts temperature in Fahrenheit to temperature in Kelvin
/// @param[in] temp_fahrenheit temperature in Fahrenheit
/// @return temperature in Kelvin
LUM_FLOATING_TEMPLATE_TYPE
constexpr T fahrenheit2kelvin(T temp_fahrenheit) noexcept
{
  return (temp_fahrenheit - static_cast<T>(32)) * static_cast<T>(5.0 / 9.0) +
         static_cast<T>(273.15);
}

/// @brief Converts temperature in Fahrenheit to temperature in Celsius
/// @param[in] temp_fahrenheit temperature in Fahrenheit
/// @return temperature in Celsius
LUM_FLOATING_TEMPLATE_TYPE
constexpr T fahrenheit2celsius(T temp_fahrenheit) noexcept
{
  return (temp_fahrenheit - static_cast<T>(32)) * static_cast<T>(5.0 / 9.0);
}

/// @brief Method to calculate mean and variance of samples in one pass
/// This is the Welford algorithm for calculating mean and variance of samples in one pass. This
/// algorithm is numerically stable. Source :
/// https://jonisalonen.com/2013/deriving-welfords-method-for-computing-variance/
/// @param[in] samples the samples
/// @return a pair of (mean, variance)
LUM_NUMERIC_TEMPLATE_TYPE
inline std::pair<double, double> WelfordOnePassMeanVariance(const std::vector<T>& samples)
{
  // if size is 0 return 0 as mean and 0 as variance
  if (samples.empty())
  {
    return std::make_pair(0.0, 0.0);
  }

  // if size is 1, return mean as the first value and variance as 0
  if (samples.size() == 1)
  {
    return std::make_pair(static_cast<double>(samples[0]), 0.0);
  }

  double mean{0};
  double variance{0};
  double old_mean{mean};
  std::size_t sample_number{1};
  for (const auto& sample : samples)
  {
    old_mean = mean;
    mean = mean + (sample - mean) / sample_number;
    variance = variance + (sample - mean) * (sample - old_mean);
    ++sample_number;
  }
  return std::make_pair(mean, variance / (samples.size() - 1));
}

} // namespace utils
} // namespace math
} // namespace common
} // namespace lum
#endif
