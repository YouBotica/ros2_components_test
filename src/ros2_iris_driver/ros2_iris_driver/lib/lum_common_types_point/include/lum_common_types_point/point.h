// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_POINT_H
#define LUM_COMMON_TYPES_POINT_H

#include <cmath>
#include <iostream>
#include <type_traits>

/// @def Template parameter restricted to numeric types
#ifndef LUM_NUMERIC_TEMPLATE_TYPE
#define LUM_NUMERIC_TEMPLATE_TYPE                                                                  \
  template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
#endif

namespace lum {
namespace common {
namespace types {
namespace point_cloud {

/// @brief Base structure for representing points
struct Point
{
};

/// @brief Structure for representing 3D points
LUM_NUMERIC_TEMPLATE_TYPE
struct alignas(16) PointXYZ : public Point
{
  PointXYZ(T x, T y, T z) noexcept : x(x), y(y), z(z) {}
  PointXYZ() noexcept = default;
  PointXYZ(const PointXYZ&) noexcept = default;
  PointXYZ(PointXYZ&&) noexcept = default;
  PointXYZ& operator=(const PointXYZ&) noexcept = default;
  PointXYZ& operator=(PointXYZ&&) noexcept = default;
  ~PointXYZ() noexcept = default;

  T x{0}; ///< X position
  T y{0}; ///< Y position
  T z{0}; ///< Z position
};

/// @brief Structure for representing 2D points
LUM_NUMERIC_TEMPLATE_TYPE
struct alignas(16) PointXY : public Point
{
  PointXY(T x, T y) noexcept : x(x), y(y) {}
  PointXY() noexcept = default;
  PointXY(const PointXY&) noexcept = default;
  PointXY(PointXY&&) noexcept = default;
  PointXY& operator=(const PointXY&) noexcept = default;
  PointXY& operator=(PointXY&&) noexcept = default;
  ~PointXY() noexcept = default;

  T x{0}; ///< X position
  T y{0}; ///< Y position
};

/// @brief Structure for representing 3D points with intensity
LUM_NUMERIC_TEMPLATE_TYPE
struct alignas(16) PointXYZI : public Point
{
  PointXYZI(T x, T y, T z, T i) noexcept : x(x), y(y), z(z), intensity(i) {}
  PointXYZI() noexcept = default;
  PointXYZI(const PointXYZI&) noexcept = default;
  PointXYZI(PointXYZI&&) noexcept = default;
  PointXYZI& operator=(const PointXYZI&) noexcept = default;
  PointXYZI& operator=(PointXYZI&&) noexcept = default;
  ~PointXYZI() noexcept = default;

  T x{0};         ///< X position
  T y{0};         ///< Y position
  T z{0};         ///< Z position
  T intensity{0}; ///< Point Intensity
};

/// @brief Structure for representing 3D points with colors
LUM_NUMERIC_TEMPLATE_TYPE
struct alignas(16) PointXYZRGB : public Point
{
  PointXYZRGB(T x, T y, T z, T r, T g, T b) noexcept : x(x), y(y), z(z), r(r), g(g), b(b) {}
  PointXYZRGB() noexcept = default;
  PointXYZRGB(const PointXYZRGB&) noexcept = default;
  PointXYZRGB(PointXYZRGB&&) noexcept = default;
  PointXYZRGB& operator=(const PointXYZRGB&) = default;
  PointXYZRGB& operator=(PointXYZRGB&&) noexcept = default;
  ~PointXYZRGB() noexcept = default;

  T x{0}; ///< X position
  T y{0}; ///< Y position
  T z{0}; ///< Z position
  T r{0}; ///< Color R value
  T g{0}; ///< Color G value
  T b{0}; ///< Color B value
};

/// @brief Structure for representing 3D points with colors
LUM_NUMERIC_TEMPLATE_TYPE
struct alignas(16) PointXYZHSV : public Point
{
  PointXYZHSV(T x, T y, T z, T h, T s, T v) noexcept : x(x), y(y), z(z), h(h), s(s), v(v) {}
  PointXYZHSV() noexcept = default;
  PointXYZHSV(const PointXYZHSV&) noexcept = default;
  PointXYZHSV(PointXYZHSV&&) noexcept = default;
  PointXYZHSV& operator=(const PointXYZHSV&) noexcept = default;
  PointXYZHSV& operator=(PointXYZHSV&&) noexcept = default;
  ~PointXYZHSV() noexcept = default;

  T x{0}; ///< X position
  T y{0}; ///< Y position
  T z{0}; ///< Z position
  T h{0}; ///< Color H value
  T s{0}; ///< Color S value
  T v{0}; ///< Color V value
};

LUM_NUMERIC_TEMPLATE_TYPE
inline std::ostream& operator<<(std::ostream& os,
                                const lum::common::types::point_cloud::PointXYZ<T>& point)
{
  os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
  return os;
}

/// @brief Function to convert HSV Point to RGB Point
/// Note: The function fills the RGB point with 8-bit color values
/// @param [in] in input HSV point with H field [0-360], S [0, 1], V[0,1]
/// @param [out] out output RGB Point
LUM_NUMERIC_TEMPLATE_TYPE
inline void pointXYZHSVtoXYZRGB(const lum::common::types::point_cloud::PointXYZHSV<T>& in,
                                lum::common::types::point_cloud::PointXYZRGB<T>& out)
{
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
  if (in.s == 0)
  {
    out.r = out.g = out.b = static_cast<std::uint8_t>(in.v);
    return;
  }
  const float a{static_cast<float>(in.h) / 60.0F};
  const std::int32_t i{static_cast<std::uint16_t>(std::floor(a))};
  const float f{a - static_cast<float>(i)};
  const float p{static_cast<float>(in.v) * (1.0F - static_cast<float>(in.s))};
  const float q{static_cast<float>(in.v) * (1.0F - static_cast<float>(in.s) * f)};
  const float t{static_cast<float>(in.v) * (1.0F - static_cast<float>(in.s) * (1.0F - f))};

  switch (i)
  {
  case 0:
  {
    out.r = static_cast<std::uint8_t>(255 * in.v);
    out.g = static_cast<std::uint8_t>(255 * t);
    out.b = static_cast<std::uint8_t>(255 * p);
    break;
  }
  case 1:
  {
    out.r = static_cast<std::uint8_t>(255 * q);
    out.g = static_cast<std::uint8_t>(255 * in.v);
    out.b = static_cast<std::uint8_t>(255 * p);
    break;
  }
  case 2:
  {
    out.r = static_cast<std::uint8_t>(255 * p);
    out.g = static_cast<std::uint8_t>(255 * in.v);
    out.b = static_cast<std::uint8_t>(255 * t);
    break;
  }
  case 3:
  {
    out.r = static_cast<std::uint8_t>(255 * p);
    out.g = static_cast<std::uint8_t>(255 * q);
    out.b = static_cast<std::uint8_t>(255 * in.v);
    break;
  }
  case 4:
  {
    out.r = static_cast<std::uint8_t>(255 * t);
    out.g = static_cast<std::uint8_t>(255 * p);
    out.b = static_cast<std::uint8_t>(255 * in.v);
    break;
  }
  default:
  {
    out.r = static_cast<std::uint8_t>(255 * in.v);
    out.g = static_cast<std::uint8_t>(255 * p);
    out.b = static_cast<std::uint8_t>(255 * q);
    break;
  }
  }
}

} // namespace point_cloud
} // namespace types
} // namespace common
} // namespace lum

#endif
