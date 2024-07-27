// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_TYPES_PREPROCESSOR_CONFIG_H
#define LUM_DRIVERS_LIDAR_IRIS_TYPES_PREPROCESSOR_CONFIG_H

#include <cstdint>
#include <unordered_set>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace types {

/// @brief Which return to process from a ray that has returns:  A specified return, or all
/// returns.
enum class ReturnFilter : std::uint32_t
{
  ALL = 0,      ///< keep all valid points
  FIRST = 1,    ///< keep points from the first return
  LAST = 2,     ///< keep points from the last return
  STRONGEST = 3 ///< keep only the strongest point from a ray
};

struct PreprocessorConfig
{
  std::uint8_t sensor_id{0U};
  std::uint16_t lines_per_frame{64U}; //< must not be zero
  std::uint16_t rays_per_line{1100U}; //< must not be zero
  bool drop_empty_rays{false};

  ReturnFilter return_filter{ReturnFilter::STRONGEST};

  // if you are dropping empty rays, then rays in the provided checkpoints
  // are also dropped.  These are as specifid in your commanded scan pattern
  std::unordered_set<std::uint8_t> filter_checkpoints{};
};

} // namespace types
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
