// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MATH_CONSTANTS_H
#define LUM_COMMON_MATH_CONSTANTS_H

#include <cmath>

namespace lum {
namespace common {
namespace math {
namespace constants {

/// @brief Returns a floating point value of 2 * PI
/// @note The c++ standard M_PI is double
static constexpr float TWO_M_PI_F{static_cast<float>(2.0 * M_PI)};

/// @brief Returns a floating point value of PI
/// @note The c++ standard M_PI is double
static constexpr float M_PI_F{static_cast<float>(M_PI)};

/// @brief Returns a floating point value of PI / 2
/// @note The c++ standard M_PI_2 is double
static constexpr float M_PI_2_F{static_cast<float>(M_PI_2)};

/// @brief Speed of light as float
static constexpr float SPEED_OF_LIGHT{299792458.0F};

/// @brief Conversion from meters per second to miles per hour
static constexpr float MPS_TO_MPH{2.23694F};

/// @brief Acceleration due to gravity in m/s^2
static constexpr float ACCELERATION_DUE_TO_GRAVITY{9.81F};

} // namespace constants
} // namespace math
} // namespace common
} // namespace lum

#endif
