// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_UTILITIES_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_UTILITIES_H

#include <cmath>
#include <cstdint>

#include <lum_common_types_internal/macros.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

constexpr float REFLECTANCE_SCALE{2048.0F};

/// @brief Convert a reflectance value from a floating-point decimal number to a fixed-point
/// unsigned Q1.11 integer
/// @param reflectance [in] a reflectance decimal value with expected range [0,2]
/// @return the unsigned Q1.11 integer that most closely represents the given input value
LUM_FLOATING_TEMPLATE_TYPE
constexpr std::uint16_t convertReflectanceToFixedPoint(T reflectance)
{
  return static_cast<std::uint16_t>(std::lround(reflectance * REFLECTANCE_SCALE));
}

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
