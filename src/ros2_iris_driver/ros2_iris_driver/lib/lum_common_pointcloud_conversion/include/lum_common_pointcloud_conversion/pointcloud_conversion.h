// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_POINTCLOUD_CONVERSION_POINTCLOUD_CONVERSION_H
#define LUM_COMMON_POINTCLOUD_CONVERSION_POINTCLOUD_CONVERSION_H

#include <lum_common_types_pointcloud/base_point_cloud.h>
#include <lum_common_types_pointcloud/point_cloud_layer.h>
#include <lum_drivers_lidar_iris_types/data_types.h>
#include <lum_drivers_lidar_iris_types/point_cloud_layer.h>
#include <lum_drivers_lidar_legacy_types/point.h>
#include <lum_drivers_lidar_legacy_types/point_cloud_layer.h>
#include <lum_semantic_segmentation_internal_types/types.h>
#include <lum_semantic_segmentation_types/point_cloud_layer.h>
#include <lum_semantic_segmentation_types/types.h>

namespace lum {
namespace common {
namespace pointcloud_conversion {

/// @brief Combines layers into legacy Hydra sensor point cloud
/// @param [in] common_layer common layer
/// @param [in] polar_layer polar layer
/// @param [in] lidar_layer lidar layer
/// @param [out] combined_cloud pointcloud constructed by combining all input layers
/// @return true if conversion is successful or false if data is not compatible
bool convertLayersToPointCloud(
  const types::point_cloud::BaseCommonLayer& common_layer,
  const types::point_cloud::BasePolarLayer& polar_layer,
  const drivers::lidar::legacy::types::point_cloud::BaseLidarLayer& lidar_layer,
  types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::Point>& combined_cloud);

/// @brief Combines Hydra layers into legacy Hydra sensor point cloud
/// @param [in] common_layer common layer
/// @param [in] polar_layer polar layer
/// @param [in] debug_layer debug layer
/// @param [in] lidar_layer lidar layer
/// @param [out] combined_cloud pointcloud constructed by combining all input layers
/// @return true if conversion is successful or false if data is not compatible
bool convertLayersToPointCloud(
  const types::point_cloud::BaseCommonLayer& common_layer,
  const types::point_cloud::BasePolarLayer& polar_layer,
  const types::point_cloud::BaseDebugLayer& debug_layer,
  const drivers::lidar::legacy::types::point_cloud::BaseLidarLayer& lidar_layer,
  types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::Point>& combined_cloud);

/// @brief Combines Iris layers into legacy Hydra sensor point cloud
/// @param [in] common_layer common layer
/// @param [in] polar_layer polar layer
/// @param [in] debug_layer debug layer
/// @param [in] lidar_layer lidar layer
/// @param [out] combined_cloud pointcloud constructed by combining all input layers
/// @return true if conversion is successful or false if data is not compatible
bool convertLayersToPointCloud(
  const types::point_cloud::BaseCommonLayer& common_layer,
  const types::point_cloud::BasePolarLayer& polar_layer,
  const types::point_cloud::BaseDebugLayer& debug_layer,
  const drivers::lidar::iris::types::point_cloud::BaseLidarLayer& lidar_layer,
  types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::Point>& combined_cloud);

/// @brief Combines layers into legacy semantic segmentation point cloud
/// @param [in] common_layer common layer
/// @param [in] polar_layer polar layer
/// @param [in] lidar_layer lidar layer
/// @param [in] semantic_layer semantic layer
/// @param [out] combined_cloud pointcloud constructed by combining all input layers
/// @return true if conversion is successful or false if data is not compatible
bool convertLayersToPointCloud(
  const types::point_cloud::BaseCommonLayer& common_layer,
  const types::point_cloud::BasePolarLayer& polar_layer,
  const drivers::lidar::legacy::types::point_cloud::BaseLidarLayer& lidar_layer,
  const semantic_segmentation::types::point_cloud::BaseSemanticSegmentationLayer& semantic_layer,
  types::point_cloud::BasePointCloud<semantic_segmentation::internal_types::ClassifiedPoint>&
    combined_cloud);

/// @brief Combines layers into legacy semantic segmentation point cloud
/// @param [in] common_layer common layer
/// @param [in] polar_layer polar layer
/// @param [in] debug_layer debug layer
/// @param [in] lidar_layer lidar layer
/// @param [in] semantic_layer semantic layer
/// @param [out] combined_cloud pointcloud constructed by combining all input layers
/// @return true if conversion is successful or false if data is not compatible
bool convertLayersToPointCloud(
  const types::point_cloud::BaseCommonLayer& common_layer,
  const types::point_cloud::BasePolarLayer& polar_layer,
  const types::point_cloud::BaseDebugLayer& debug_layer,
  const drivers::lidar::legacy::types::point_cloud::BaseLidarLayer& lidar_layer,
  const semantic_segmentation::types::point_cloud::BaseSemanticSegmentationLayer& semantic_layer,
  types::point_cloud::BasePointCloud<semantic_segmentation::internal_types::ClassifiedPoint>&
    combined_cloud);

/// @brief Combines layers into legacy semantic segmentation point cloud
/// @param [in] common_layer common layer
/// @param [in] semantic_layer semantic layer
/// @param [out] combined_cloud pointcloud constructed by combining all input layers
/// @return true if conversion is successful or false if data is not compatible
bool convertLayersToPointCloud(
  const types::point_cloud::BaseCommonLayer& common_layer,
  const semantic_segmentation::types::point_cloud::BaseSemanticSegmentationLayer& semantic_layer,
  types::point_cloud::BasePointCloud<semantic_segmentation::internal_types::ClassifiedPoint>&
    combined_cloud);

/// @brief Extracts common layer from legacy Hydra sensor point cloud
/// @param [in] combined_cloud Hydra sensor point cloud
/// @param [out] common_layer common layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractCommonLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::Point>& combined_cloud,
  types::point_cloud::BaseCommonLayer& common_layer);

/// @brief Extracts polar layer from legacy Hydra sensor point cloud
/// @param [in] combined_cloud Hydra sensor point cloud
/// @param [out] polar_layer polar layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractPolarLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::Point>& combined_cloud,
  types::point_cloud::BasePolarLayer& polar_layer);

/// @brief Extracts debug layer from legacy Hydra sensor point cloud
/// @param [in] combined_cloud Hydra sensor point cloud
/// @param [out] debug_layer debug layer to extract information to4
/// @return true if extraction is successful or false if data is not compatible
bool extractDebugLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::Point>& combined_cloud,
  types::point_cloud::BaseDebugLayer& debug_layer);

/// @brief Extracts lidar layer from legacy Hydra sensor point cloud
/// @param [in] combined_cloud Hydra sensor point cloud
/// @param [out] lidar_layer lidar layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractLidarLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::Point>& combined_cloud,
  drivers::lidar::legacy::types::point_cloud::BaseLidarLayer& lidar_layer);

/// @brief Extracts lidar layer from legacy Iris sensor point cloud
/// @param [in] combined_cloud Hydra sensor point cloud
/// @param [out] lidar_layer lidar layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractLidarLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::Point>& combined_cloud,
  drivers::lidar::iris::types::point_cloud::BaseLidarLayer& lidar_layer);

/// @brief Extracts semantic layer from legacy semantic segmentation point cloud
/// @param [in] combined_cloud semantic segmentation point cloud
/// @param [out] semantic_layer semantic layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractSemanticLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<semantic_segmentation::internal_types::ClassifiedPoint>&
    combined_cloud,
  semantic_segmentation::types::point_cloud::BaseSemanticSegmentationLayer& semantic_layer);

/// @brief Extracts common layer from legacy Hydra sensor line data
/// @param [in] line Hydra sensor line point cloud
/// @param [in] metadata Hydra sensor point cloud line metadata
/// @param [out] common_layer common layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractCommonLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::LinePoint>& line,
  const drivers::lidar::legacy::types::PointCloudLineMeta& metadata,
  types::point_cloud::BaseCommonLayer& common_layer);

/// @brief Extracts polar layer from legacy Hydra sensor line data
/// @param [in] line Hydra sensor line point cloud
/// @param [out] polar_layer polar layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractPolarLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::LinePoint>& line,
  types::point_cloud::BasePolarLayer& polar_layer);

/// @brief Extracts debug layer from legacy Hydra sensor line data
/// @param [in] line Hydra sensor line point cloud
/// @param [out] debug_layer debug layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractDebugLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::LinePoint>& line,
  types::point_cloud::BaseDebugLayer& debug_layer);

/// @brief Extracts lidar layer from legacy Hydra sensor line data
/// @param [in] line Hydra sensor line point cloud
/// @param [in] metadata Hydra sensor point cloud line metadata
/// @param [out] lidar_layer lidar layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractLidarLayerFromPointCloud(
  const types::point_cloud::BasePointCloud<drivers::lidar::legacy::types::LinePoint>& line,
  const drivers::lidar::legacy::types::PointCloudLineMeta& metadata,
  drivers::lidar::iris::types::point_cloud::BaseLidarLayer& lidar_layer);

/// @brief Extracts common layer from legacy Hydra sensor line data w/ supplemental fields
/// @param [in] line_with_supp_fields Hydra sensor line w/ supplemental fields
/// @param [out] common_layer common layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractCommonLayerFromPointCloud(
  const drivers::lidar::iris::types::LinePointsWithSupplementalFields& line_with_supp_fields,
  types::point_cloud::BaseCommonLayer& common_layer);

/// @brief Extracts polar layer from legacy Hydra sensor line data w/ supplemental fields
/// @param [in] line_with_supp_fields Hydra sensor line w/ supplemental fields
/// @param [out] polar_layer polar layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractPolarLayerFromPointCloud(
  const drivers::lidar::iris::types::LinePointsWithSupplementalFields& line_with_supp_fields,
  types::point_cloud::BasePolarLayer& polar_layer);

/// @brief Extracts debug layer from legacy Hydra sensor line data w/ supplemental fields
/// @param [in] line_with_supp_fields Hydra sensor line w/ supplemental fields
/// @param [out] debug_layer debug layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractDebugLayerFromPointCloud(
  const drivers::lidar::iris::types::LinePointsWithSupplementalFields& line_with_supp_fields,
  types::point_cloud::BaseDebugLayer& debug_layer);

/// @brief Extracts lidar layer from legacy Hydra sensor line data w/ supplemental fields
/// @param [in] line_with_supp_fields Hydra sensor line w/ supplemental fields
/// @param [out] lidar_layer lidar layer to extract information to
/// @return true if extraction is successful or false if data is not compatible
bool extractLidarLayerFromPointCloud(
  const drivers::lidar::iris::types::LinePointsWithSupplementalFields& line_with_supp_fields,
  drivers::lidar::iris::types::point_cloud::BaseLidarLayer& lidar_layer);

} // namespace pointcloud_conversion
} // namespace common
} // namespace lum

#endif
