// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_POINT_CLOUD_LAYER_H
#define LUM_COMMON_TYPES_POINT_CLOUD_LAYER_H

#include <lum_common_types/units.h>
#include <lum_common_types_pointcloud/base_point_cloud.h>
#include <lum_common_types_pointcloud/structured_point_cloud.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>

namespace lum {
namespace common {
namespace types {
namespace point {

/// @brief Structure to represent generic 3D point information
struct CommonData
{
  /// @brief The lidar system time
  /// @details uses PTP if available; otherwise, the value represents the amount of time since
  /// sensor bootup.
  Time timestamp{};

  /// @brief Coordinate x in cartesian space in meters
  types::units::DistanceInMeters<float> x{0.0F};

  /// @brief Coordinate y in cartesian space in meters
  types::units::DistanceInMeters<float> y{0.0F};

  /// @brief Coordinate z in cartesian space in meters
  types::units::DistanceInMeters<float> z{0.0F};

  /// @brief The estimated reflectance of the target; valid range: 0-2; see lidar documentation for
  /// more information
  float reflectance{0.0F};

  /// @brief The return index of this point (or zero if there is no valid return)
  /// @details Each laser pulse that is sent out can encounter objects in its path, and the sensor
  /// records returns of objects that it encounters. Valid returns are indexed starting at 1, and an
  /// index of 0 means that the current pulse had no returns (i.e., it did not hit a reflective
  /// object).
  std::uint8_t return_index{0U};

  /// @brief The index of the last return (or zero if no returns)
  /// @details Each laser pulse that is sent out can encounter multiple objects in its path and
  /// record returns or objects that it encounters. This value represents the total number of
  /// returns received for a given ray (i.e., the last valid index).
  std::uint8_t last_return_index{0U};

  /// @brief Sensor ID this point was originated from (useful in multi-sensor configurations)
  std::uint8_t sensor_id{0U};
};

/// @brief helper function to detect if this is an empty point or a real return from the LiDAR
/// @return true if this point is empty
inline bool isEmpty(const CommonData& p)
{
  return p.return_index == 0U;
}

/// @brief Structure to represent point coordinates in the polar coordinate system
struct PolarData
{
  /// @brief Azimuth coordinate in spherical space in radians
  types::units::AngleInRadians<float> azimuth{0.0F};
  /// @brief Elevation coordinate in spherical space in radians
  types::units::AngleInRadians<float> elevation{0.0F};
  /// @brief Depth coordinate in spherical space in meters
  types::units::DistanceInMeters<float> depth{0.0F};
};

/// @brief Structure to represent debugging information from the lidar
struct DebugData
{
  /// @brief The number of bytes in the reserved field
  static constexpr std::size_t NUM_RESERVED_FIELDS{4U};

  /// @brief The reserved bytes received from the lidar; contents are undefined and subject to
  /// change
  std::uint8_t reserved[NUM_RESERVED_FIELDS]{0U};
};

} // namespace point

namespace point_cloud {
/// @brief Structure to represent a common layer of the base pointcloud
using BaseCommonLayer = BasePointCloud<point::CommonData>;

/// @brief Structure to represent a common layer of the unstructured pointcloud
using UnstructuredCommonLayer = UnstructuredPointCloud<point::CommonData>;

/// @brief Structure to represent a common layer of the structured pointcloud
using StructuredCommonLayer = StructuredPointCloud<point::CommonData>;

/// @brief Structure to represent a polar layer of the base pointcloud
using BasePolarLayer = BasePointCloud<point::PolarData>;

/// @brief Structure to represent a polar coordinates layer of the unstructured pointcloud
using UnstructuredPolarLayer = UnstructuredPointCloud<point::PolarData>;

/// @brief Structure to represent a polar coordinates layer of the structured pointcloud
using StructuredPolarLayer = StructuredPointCloud<point::PolarData>;

/// @brief Structure to represent a debugging layer of the base pointcloud
using BaseDebugLayer = BasePointCloud<point::DebugData>;

/// @brief Structure to represent a debugging layer of the unstructured pointcloud
using UnstructuredDebugLayer = UnstructuredPointCloud<point::DebugData>;

/// @brief Structure to represent a debugging layer of the structured pointcloud
using StructuredDebugLayer = StructuredPointCloud<point::DebugData>;

/// @brief This function checks if 2 pointcloud layers are matching so they belong to the same
/// pointcloud.
/// @param[in] layer_a first point cloud layer
/// @param[in] layer_b second point cloud layer
/// @return true if layers are matching
template <template <typename> class CloudT,
          typename PointT1,
          typename PointT2,
          typename = typename std::enable_if<
            std::is_base_of<BasePointCloud<PointT1>, CloudT<PointT1>>::value>::type>
bool areMatching(const CloudT<PointT1>& layer_a, const CloudT<PointT2>& layer_b)
{
  const bool same_size = layer_a.size() == layer_b.size();
  const bool same_timestamp = layer_a.getTimestamp() == layer_b.getTimestamp();
  return same_timestamp && same_size;
}
} // namespace point_cloud
} // namespace types
} // namespace common
} // namespace lum

#endif
