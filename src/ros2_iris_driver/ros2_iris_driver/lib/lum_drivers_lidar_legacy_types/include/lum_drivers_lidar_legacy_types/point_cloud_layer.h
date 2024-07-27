// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PDK_DRIVERS_LIDAR_HYDRA_TYPES_POINT_CLOUD_LAYER_H
#define LUM_PDK_DRIVERS_LIDAR_HYDRA_TYPES_POINT_CLOUD_LAYER_H

#include <cstdint>

#include <lum_common_types_pointcloud/base_point_cloud.h>
#include <lum_common_types_pointcloud/structured_point_cloud.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace legacy {
namespace types {
namespace point {

/// @brief Structure to represent point information for the common pointcloud layer
struct LidarData
{
  /// Index of the scan line
  std::int16_t scan_line{0};
  /// Lidar eye (left or right)
  std::uint8_t eye{0U};
  /// Interlace index
  std::uint8_t interlace_index{0U};
  /// Track index within scan playlist
  std::uint8_t track_number{0U};
  /// Checkpoint index within scan pattern
  std::uint8_t checkpoint{0U};
};

} // namespace point

namespace point_cloud {
/// @brief Structure to represent a hydra lidar layer of the base pointcloud
using BaseLidarLayer = lum::common::types::point_cloud::BasePointCloud<point::LidarData>;

/// @brief Structure to represent a hydra lidar layer of the unstructured pointcloud
using UnstructuredLidarLayer =
  lum::common::types::point_cloud::UnstructuredPointCloud<point::LidarData>;

/// @brief Structure to represent a hydra lidar layer of the structured pointcloud
using StructuredLidarLayer =
  lum::common::types::point_cloud::StructuredPointCloud<point::LidarData>;

} // namespace point_cloud
} // namespace types
} // namespace legacy
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
