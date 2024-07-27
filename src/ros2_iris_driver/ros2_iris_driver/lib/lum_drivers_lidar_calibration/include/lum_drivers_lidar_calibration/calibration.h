// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_CALIBRATION_CALIBRATION_H
#define LUM_DRIVERS_LIDAR_CALIBRATION_CALIBRATION_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_set>

#include <lum_common_math/floating_point.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace legacy {
namespace types {
/// Lidar Point filter type, filters points by type specifiers from the originating ray
/// Used to include / omit entire rays
/// @note filtering snapback points should be done at the line-level, using
/// PointCloudLineMeta::isSnapback
enum class PointTypeFilter : std::uint8_t
{
  KEEP_VALID = 0,     ///< keep points where at least one return was valid
  KEEP_ALL_POINTS = 1 ///< no filtering, invalid, empty
};

/// Lidar return filter type
/// Used during conversion from ray return to point.
/// @note if a ray has no returns, then it can emit an empty point according to RayFilter setting
enum class MultiReturnFilter : std::uint8_t
{
  ALL = 0,      ///< keep all valid points
  FIRST = 1,    ///< keep points from the first return
  LAST = 2,     ///< keep points from the last return
  STRONGEST = 3 ///< keep only the strongest point from a ray
};

/// enum to specify how to divide a data stream into frames
///
/// Scan frames are a nested structure with Hydra:
///
/// scan_count
/// | interlace
/// | | playlist
/// | | -
/// | -
/// -
///
/// We always split on scan_count boundary, we can optionally also split at interlace (which
/// includes scan_count), or on playlist track (which includes interlace and scan_count)
///
enum class FrameBoundarySplitMode : std::uint8_t
{
  SCAN_ONLY = 0,                         ///< split only on scan_count
  SCAN_AND_INTERLACING = 1,              ///< split on scan count and interlace index
  SCAN_AND_INTERLACING_AND_PLAYLIST = 2, ///< split on scan count and playlist track
};
} // namespace types
} // namespace legacy

///////////////////////////////////////////////////////////////////////////////////////////////////
namespace calibration {

using legacy::types::MultiReturnFilter;
using legacy::types::PointTypeFilter;

constexpr std::size_t NUM_EYES{2U};
constexpr std::int32_t DEFAULT_NUM_LINES{64};
constexpr std::int32_t DEFAULT_POINTS_PER_EYE{500};

////////////////////////////////////////////////////////////////////////////

/// Structure to represent LiDAR calibration parameters specific to each eye
struct EyeCalibrationParameters
{
  float elevation_scale{1.0F};  ///< Scale factor for elevation
  float azimuth_scale{1.0F};    ///< Scale factor for azimuth
  float distance_scale{1.0F};   ///< Scale factor for distance
  float elevation_offset{0.0F}; ///< Offset factor for elevation
  float azimuth_offset{0.0F};   ///< Offset factor for azimuth
  float distance_offset{0.0F};  ///< Offset factor for distance
};

/// true if parameters match
inline bool operator==(const EyeCalibrationParameters& a, const EyeCalibrationParameters& b)
{
  using common::math::areApproximatelyEqual;
  return areApproximatelyEqual(a.elevation_scale, b.elevation_scale) &&
         areApproximatelyEqual(a.azimuth_scale, b.azimuth_scale) &&
         areApproximatelyEqual(a.distance_scale, b.distance_scale) &&
         areApproximatelyEqual(a.elevation_offset, b.elevation_offset) &&
         areApproximatelyEqual(a.azimuth_offset, b.azimuth_offset) &&
         areApproximatelyEqual(a.distance_offset, b.distance_offset);
}

/// true if calibration is default
inline bool isDefaultEyeParam(const EyeCalibrationParameters& p)
{
  return p == EyeCalibrationParameters{};
}

////////////////////////////////////////////////////////////////////////////

/// Structure to represent LiDAR calibration parameters
struct CalibrationParameters
{
  /// Expected number of scan lines
  std::int32_t num_lines{DEFAULT_NUM_LINES};

  /// Expected number of points in each scan line
  std::int32_t points_per_eye{DEFAULT_POINTS_PER_EYE};

  /// Mode for filtering points from empty rays
  PointTypeFilter lidar_point{PointTypeFilter::KEEP_ALL_POINTS};

  /// Method for choosing points by return within rays
  MultiReturnFilter lidar_return{MultiReturnFilter::STRONGEST};

  /// Method for choosing where to split data into frames
  /// @note this is currently only used for Hydra sensorssnap
  legacy::types::FrameBoundarySplitMode frame_split_mode{
    legacy::types::FrameBoundarySplitMode::SCAN_ONLY};

  /// Whether structured point cloud lines are sorted vertically within a frame instead of
  /// presented in sensor order. (Not supported on Iris.)
  bool sort_scan_lines_vertically{false};

  /// Whether structured point cloud lines are sorted horizontally within a frame.
  /// (Not supported on Iris.)
  bool sort_scan_lines_horizontally{false};

  /// Minimum depth for filtering out very close points
  float min_depth{0.0F};

  /// Maximum depth for filtering out very far points
  float max_depth{300.0F};

  /// Calibration parameters specific to each eye
  std::array<EyeCalibrationParameters, static_cast<std::size_t>(NUM_EYES)> eyes{};

  /// Specified checkpoints to drop while filtering by checkpoint, or snapback
  /// @note this is currently only used for Iris sensors
  std::unordered_set<std::uint8_t> snap_checkpoints{};
};

/// Comparison operator
inline bool operator==(const CalibrationParameters& a, const CalibrationParameters& b)
{
  using common::math::areApproximatelyEqual;
  return a.num_lines == b.num_lines &&                                       //
         a.points_per_eye == b.points_per_eye &&                             //
         a.lidar_point == b.lidar_point &&                                   //
         a.lidar_return == b.lidar_return &&                                 //
         a.frame_split_mode == b.frame_split_mode &&                         //
         a.sort_scan_lines_vertically == b.sort_scan_lines_vertically &&     //
         a.sort_scan_lines_horizontally == b.sort_scan_lines_horizontally && //
         areApproximatelyEqual(a.min_depth, b.min_depth) &&                  //
         areApproximatelyEqual(a.max_depth, b.max_depth) &&                  //
         a.eyes == b.eyes &&                                                 //
         a.snap_checkpoints == b.snap_checkpoints;
}

/// true if calibration is default
inline bool isDefault(const CalibrationParameters& c)
{
  return c == CalibrationParameters{};
}

} // namespace calibration
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
