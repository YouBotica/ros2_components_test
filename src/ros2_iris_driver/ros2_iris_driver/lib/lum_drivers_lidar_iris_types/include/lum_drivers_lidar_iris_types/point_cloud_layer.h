// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_TYPES_POINT_CLOUD_LAYER_H
#define LUM_DRIVERS_LIDAR_IRIS_TYPES_POINT_CLOUD_LAYER_H

#include <cstdint>

#include <lum_common_types_pointcloud/base_point_cloud.h>
#include <lum_common_types_pointcloud/point_cloud_layer.h>
#include <lum_common_types_pointcloud/structured_point_cloud.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>
#include <lum_drivers_lidar_iris_types/data_types.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace types {
namespace point {

/// @brief Structure to represent point information for the lidar model-specific pointcloud layer
struct LidarData
{
  /// @brief The line index within the current frame; wraps back to 0 if the maximum value is
  /// reached; valid range: 0-511
  std::uint16_t line_index{0U};

  /// @brief The frame index within the data stream; wraps back to 0 if the maximum value is reached
  /// (hence it is not a sufficient unique identifier for long sequences); valid range: 0-255
  std::uint8_t frame_index{0U};

  /// @brief The detector site identifier for this lidar ray, i.e. Site A or Site B
  DetectorSiteId detector_site_id{};

  /// @brief The most recent checkpoint value within the active scan pattern; these values are
  /// user-defined within the scan control interface; valid range: 0-255
  std::uint8_t scan_checkpoint{0U};

  /// @brief The likelihood that the return is from a real world object (i.e. not noise);
  /// valid range: 0-255 (representing 0-100%)
  std::uint8_t existence_probability_percent{0U};

  /// @brief The self-assessment by the sensor of the validity of the lidar data
  DataQualifier data_qualifier{};

  /// @brief A determination of a given ray's blockage level from 0 to 15, where 15 is maximum
  /// blockage.
  std::uint8_t blockage_level{0U};
};

struct CombinedData
{
  // CommonData
  lum::common::types::point::CommonData common_data;

  // PolarData
  lum::common::types::point::PolarData polar_data;

  // LidarData
  LidarData lidar_data;
};

} // namespace point

namespace point_cloud {
/// @brief Structure to represent an iris lidar layer of the base pointcloud
using BaseLidarLayer = common::types::point_cloud::BasePointCloud<point::LidarData>;

/// @brief Structure to represent an iris lidar layer of the unstructured pointcloud
using UnstructuredLidarLayer = common::types::point_cloud::UnstructuredPointCloud<point::LidarData>;

/// @brief Structure to represent an iris lidar layer of the structured pointcloud
using StructuredLidarLayer = common::types::point_cloud::StructuredPointCloud<point::LidarData>;

/// @brief Structure to represent an iris lidar layer of the unstructured pointcloud
using UnstructuredCombinedLayer =
  common::types::point_cloud::UnstructuredPointCloud<point::CombinedData>;
/// @brief Structure to represent an iris lidar layer of the structured pointcloud
using StructuredCombinedLayer =
  common::types::point_cloud::StructuredPointCloud<point::CombinedData>;

/// @brief Structure that aggregates the Iris point layers
struct UnstructuredLayeredData
{
  common::types::point_cloud::UnstructuredCommonLayer common_layer;
  common::types::point_cloud::UnstructuredPolarLayer polar_layer;
  UnstructuredLidarLayer lidar_layer;
  common::types::point_cloud::UnstructuredDebugLayer debug_layer;
};

/// @brief Structure that aggregates the Iris structured point layers
struct StructuredLayeredData
{
  common::types::point_cloud::StructuredCommonLayer common_layer;
  common::types::point_cloud::StructuredPolarLayer polar_layer;
  StructuredLidarLayer lidar_layer;
  common::types::point_cloud::StructuredDebugLayer debug_layer;
};

} // namespace point_cloud

} // namespace types
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
