// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_INTERNAL_KINEMATICS_H
#define LUM_COMMON_TYPES_INTERNAL_KINEMATICS_H

#include <lum_common_geometry_types/geometry_types.h>
#include <lum_common_types_internal/macros.h>
#include <lum_common_types_point/point.h>

namespace lum {
namespace common {
namespace types {
namespace kinematics {

/// @brief Structure to represent velocity broken into linear and angular parts
LUM_FLOATING_TEMPLATE_TYPE
struct Twist
{
  lum::common::types::geometry::Vector3<T> linear;  ///< Linear velocity  x, y, and z components
  lum::common::types::geometry::Vector3<T> angular; ///< Angular velocity x, y, and z components
};

/// @brief Structure to represent velocity broken into linear and angular parts,
/// and the covariance of the velocity components
LUM_FLOATING_TEMPLATE_TYPE
struct TwistWithCovariance
{

  Twist<T> twist; ///< Linear and Angular velocity
  std::array<T, lum::common::types::geometry::detail::CalcNumCovarianceDimensions<6>::value>
    covariance; ///< Covarinace of velocity components
};

/// @brief Structure to represent acceleration broken into linear and angular
/// parts
LUM_FLOATING_TEMPLATE_TYPE
struct Acceleration
{
  lum::common::types::geometry::Vector3<T> linear; ///< Linear acceleration  x, y, and z components
  lum::common::types::geometry::Vector3<T>
    angular; ///< Angular acceleration  x, y, and z components
};

/// @brief Structure to represent acceleration broken into linear and angular
/// parts, and the covariance of the acceleration components
LUM_FLOATING_TEMPLATE_TYPE
struct AccelarationWithCovariance
{
  Acceleration<T> acceleration; ///< Linear and Angular acceleration
  std::array<T, lum::common::types::geometry::detail::CalcNumCovarianceDimensions<6>::value>
    covariance; ///< Covariance of acceleration components
};

} // namespace kinematics
} // namespace types
} // namespace common
} // namespace lum

#endif
