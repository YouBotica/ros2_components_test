// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MATH_VECTOR_H
#define LUM_COMMON_MATH_VECTOR_H

#include <cmath>

namespace lum {
namespace common {
namespace math {
namespace vector3d {

/// @brief Check for zero length vector
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @param [in] vector
/// @return true if vector's length is zero
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline bool isZero(const VectorType& vector)
{
  constexpr auto ZERO = static_cast<CoordinateType>(0);
  return vector.x == ZERO && vector.y == ZERO && vector.z == ZERO;
}

/// @brief Check if vector is finite
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @param [in] vector
/// @return true if vector contain correct finite numbers
template <typename VectorType>
inline bool isFinite(const VectorType& vector)
{
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

/// @brief Check if vector is correct: has finite values and non zero length
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @param [in] vector
/// @return true if vector is correct
template <typename VectorType>
inline bool isCorrect(const VectorType& vector)
{
  return isFinite<VectorType>(vector) && !isZero<VectorType>(vector);
}

/// @brief Set vector coordinates
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] vector
/// @param [in] x coordinate component
/// @param [in] y coordinate component
/// @param [in] z coordinate component
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void
set(VectorType& vector, const CoordinateType x, const CoordinateType y, const CoordinateType z)
{
  vector.x = x;
  vector.y = y;
  vector.z = z;
}

/// @brief Initialize vector coordinates with the same value
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] vector
/// @param [in] val value to be used to set x, y, z components
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void set(VectorType& vector, const CoordinateType val)
{
  set<VectorType, CoordinateType>(vector, val, val, val);
}

/// @brief Copy x, y, z coordinates of source vector to target vector
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] target vector to receive data
/// @param [in] source vector to copy data from
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void copy(VectorType& target, const VectorType& source)
{
  set<VectorType, CoordinateType>(target, source.x, source.y, source.z);
}

/// @brief Add 2 vectors (target = a + b)
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] target vector to receive result
/// @param [in] a first operand vector
/// @param [in] b second operand vector
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void add(VectorType& target, const VectorType& a, const VectorType& b)
{
  set<VectorType, CoordinateType>(target, a.x + b.x, a.y + b.y, a.z + b.z);
}

/// @brief Subtract 2 vectors (target = a - b)
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] target vector to receive result
/// @param [in] a first operand vector
/// @param [in] b second operand vector
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void subtract(VectorType& target, const VectorType& a, const VectorType& b)
{
  set<VectorType, CoordinateType>(target, a.x - b.x, a.y - b.y, a.z - b.z);
}

/// @brief Multiply vector by a scalar value (vector = vector * val)
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] vector to be multiplied
/// @param [in] val scalar multiplier
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void multiplyScalar(VectorType& vector, const CoordinateType val)
{
  set<VectorType, CoordinateType>(vector, vector.x * val, vector.y * val, vector.z * val);
}

/// @brief Divide vector by a scalar value (vector = vector * val)
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] vector to be multiplied
/// @param [in] val scalar divider
/// @note If val is zero - the function will do nothing
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void divideScalar(VectorType& vector, const CoordinateType val)
{
  if (val != static_cast<CoordinateType>(0))
  {
    multiplyScalar<VectorType, CoordinateType>(vector, static_cast<CoordinateType>(1) / val);
  }
}

/// @brief Getting cross product of 2 vectors (target = a x b)
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] target vector to receive result
/// @param [in] a first operand vector
/// @param [in] b second operand vector
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void getCrossProduct(VectorType& target, const VectorType& a, const VectorType& b)
{
  set<VectorType, CoordinateType>(
    target, a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

/// @brief Getting dot product of 2 vectors (target = a * b)
/// @tparam [in] ReturnType type of return value.
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @param [in] a first operand vector
/// @param [in] b second operand vector
/// @return dot product
template <typename ReturnType = double, typename VectorType>
inline ReturnType getDotProduct(const VectorType& a, const VectorType& b)
{
  return static_cast<ReturnType>(a.x) * static_cast<ReturnType>(b.x) +
         static_cast<ReturnType>(a.y) * static_cast<ReturnType>(b.y) +
         static_cast<ReturnType>(a.z) * static_cast<ReturnType>(b.z);
}

/// @brief Getting squared length of a vector
/// @tparam [in] ReturnType type of return value.
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @param [in] vector
/// @return vector length (squared)
template <typename ReturnType = double, typename VectorType>
inline ReturnType getLengthSquared(const VectorType& vector)
{
  const ReturnType x{static_cast<ReturnType>(vector.x)};
  const ReturnType y{static_cast<ReturnType>(vector.y)};
  const ReturnType z{static_cast<ReturnType>(vector.z)};
  return x * x + y * y + z * z;
}

/// @brief Getting length of a vector
/// @tparam [in] ReturnType type of return value.
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @param [in] vector
/// @return vector length
template <typename ReturnType = double, typename VectorType>
inline ReturnType getLength(const VectorType& vector)
{
  return std::sqrt(getLengthSquared<ReturnType>(vector));
}

/// @brief Normalize a vector
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @tparam [in] CoordinateType type of x, y and z members of VectorType struct.
/// @param [out] vector
template <typename VectorType, typename CoordinateType = decltype(VectorType::x)>
inline void normalize(VectorType& vector)
{
  const auto len = getLength<CoordinateType>(vector);
  divideScalar<VectorType>(vector, len);
}

/// @brief Invert a vector
/// @tparam [in] VectorType struct. It must contain public members x, y and z.
/// @param [out] vector
template <typename VectorType>
inline void invert(VectorType& a)
{
  multiplyScalar<VectorType>(a, -1);
}

} // namespace vector3d

namespace point3d {

/// @brief Getting squared distance between 2 points
/// @tparam [in] ReturnType type return value.
/// @tparam [in] PointType struct. It must contain public members x, y and z.
/// @param [in] a first point
/// @param [in] b second point
/// @return distance (squared)
template <typename ReturnType = double, typename PointType>
inline ReturnType getDistanceSquared(const PointType& a, const PointType& b)
{
  const ReturnType x{static_cast<ReturnType>(a.x - b.x)};
  const ReturnType y{static_cast<ReturnType>(a.y - b.y)};
  const ReturnType z{static_cast<ReturnType>(a.z - b.z)};
  return x * x + y * y + z * z;
}

/// @brief Getting distance between 2 points
/// @tparam [in] ReturnType type return value.
/// @tparam [in] PointType struct. It must contain public members x, y and z.
/// @param [in] a first point
/// @param [in] b second point
/// @return distance
template <typename ReturnType = double, typename PointType>
inline ReturnType getDistance(const PointType& a, const PointType& b)
{
  return std::sqrt(getDistanceSquared<ReturnType>(a, b));
}

} // namespace point3d
} // namespace math
} // namespace common
} // namespace lum

#endif // LUM_COMMON_MATH_VECTOR_H
