// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MATH_COORDINATES_H
#define LUM_COMMON_MATH_COORDINATES_H

#include <cmath>

namespace lum {
namespace common {
namespace math {

/// @brief convert spherical coordinates to cartesian, and assign them to a point
/// @param [in] elevation spherical elevation coordinate
/// @param [in] azimuth spherical azimuth coordinate
/// @param [in] range spherical range coordinate
/// @param [out] point a templated point with float x, y, and z coordinates
template <typename T>
inline void sphericalToCartesian(float elevation, float azimuth, float range, T& point)
{
  const float d_cos_el = range * std::cos(elevation);
  point.x = d_cos_el * std::cos(azimuth); // d * std::cos(az) * std::cos(el);
  point.y = d_cos_el * std::sin(azimuth); // d * std::sin(az) * std::cos(el)
  point.z = range * std::sin(elevation);
}

} // namespace math
} // namespace common
} // namespace lum

#endif
