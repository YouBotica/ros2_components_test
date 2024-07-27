// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_REGION_OF_INTEREST_H
#define LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_REGION_OF_INTEREST_H

#include <lum_common_utils/clamp.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace resim {

constexpr float MAX_AZ_VALUE{1.570796327F};
constexpr float MIN_AZ_VALUE{-1.570796327F};
constexpr float MAX_EL_VALUE{1.570796327F};
constexpr float MIN_EL_VALUE{-1.570796327F};

/// @brief A wrapping class to expose Region-of-Interest controls through iris data sensors
class IrisRegionOfInterest
{
public:
  /// @brief Set the minimum azimuth.  0 is straight on, negative values are to the left, positive
  /// values are to the right.  This is expected to be smaller than the maximum azimuth
  /// @param [in] new_azimuth the new min azimuth to use
  /// @note This value is clamped when set
  void setMinAzimuth(float new_azimuth)
  {
    min_azimuth_ = common::utils::clamp(new_azimuth, MIN_AZ_VALUE, MAX_AZ_VALUE);
  }

  /// @brief Get the current minimum azimuth
  /// @return The configured value
  float getMinAzimuth() const { return min_azimuth_; }

  /// @brief Set the maximum azimuth.  0 is straight on, negative values are to the left, positive
  /// values are to the right.  This is expected to be larger than the minimum azimuth
  /// @param [in] new_azimuth the new max azimuth to use
  /// @note This value is clamped when set
  void setMaxAzimuth(float new_azimuth)
  {
    max_azimuth_ = common::utils::clamp(new_azimuth, MIN_AZ_VALUE, MAX_AZ_VALUE);
  }

  /// @brief Get the current maximum azimuth from this config object
  /// @return The configured maximum azimuth
  float getMaxAzimuth() const { return max_azimuth_; }

  /// @brief Set the minimum elevation.  0 is straight on, negative values are down, positive values
  /// are up.  This is expected to be smaller than the maximum elevation
  /// @param [in] new_elevation the new min elevation to use
  /// @note This value is clamped when set
  void setMinElevation(float new_elevation)
  {
    min_elevation_ = common::utils::clamp(new_elevation, MIN_EL_VALUE, MAX_EL_VALUE);
  }

  /// @brief Get the current min elevation from this config object
  /// @return The current min elevation
  float getMinElevation() const { return min_elevation_; }

  /// @brief Set the maximum elevation.  0 is straight on, negative values are down, positive values
  /// are up.  This is expected to be larger than the minimum elevation
  /// @param [in] new_elevation the new max elevation to use
  /// @note This value is clamped when set
  void setMaxElevation(float new_elevation)
  {
    max_elevation_ = common::utils::clamp(new_elevation, MIN_EL_VALUE, MAX_EL_VALUE);
  }

  /// @brief Get the currently set max elevation from this config object
  /// @return The max elevation
  float getMaxElevation() const { return max_elevation_; }

private:
  float min_azimuth_{MIN_AZ_VALUE};
  float max_azimuth_{MAX_AZ_VALUE};
  float min_elevation_{MIN_EL_VALUE};
  float max_elevation_{MAX_EL_VALUE};
};

} // namespace resim
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_REGION_OF_INTEREST_H
