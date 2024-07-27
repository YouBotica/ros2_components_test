// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_GEOMETRY_H
#define LUM_COMMON_TYPES_GEOMETRY_H

namespace lum {
namespace common {
namespace types {
namespace geometry {

/// @namespace lum::common::types::geometry Geometry data types

///@brief A primitive struct for representing 2-d data.
///
/// Initialized at the origin
struct Vector2f
{
  float x{0.0F};
  float y{0.0F};
};

///@brief A primitive struct for 3-dof translation or rotation definition.
///
/// Initialized at the origin
struct Vector3f
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

///@brief A primitive struct for quaternion definition.
///
/// Initialized as identity quaternion.
struct Quaternionf
{
  float w{1.0F};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

/// @brief Pose
struct Posef
{
  Quaternionf orientation{}; ///< Orientation as quaternion
  Vector3f position{};       ///< Position stored as x,y,z
};

} // namespace geometry
} // namespace types
} // namespace common
} // namespace lum

#endif
