// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_SEMANTIC_SEGMENTATION_TYPES_POINT_CLOUD_LAYER_H
#define LUM_SEMANTIC_SEGMENTATION_TYPES_POINT_CLOUD_LAYER_H

#include <lum_common_types_classification/ontology.h>
#include <lum_common_types_pointcloud/structured_point_cloud.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>

namespace lum {
namespace semantic_segmentation {
namespace types {
namespace point {

constexpr std::size_t NUM_POINT_CLASSIFICATIONS{3};
constexpr std::uint32_t UNKNOWN_INSTANCE_ID{std::numeric_limits<std::uint32_t>::max()};

/// @brief Structure to represent point information for the common pointcloud layer
struct SemanticSegmentationData
{
  /// @brief Array of classifications, in order from most to least confident, i.e. entry 0 will be
  /// the top classification, entry 1 will be the second best classification
  lum::common::types::classification::ClassificationWithConfidence
    classifications[NUM_POINT_CLASSIFICATIONS];

  ///< ID of the cluster to which this point belongs
  std::uint32_t instance_id{UNKNOWN_INSTANCE_ID};
};

} // namespace point

namespace point_cloud {
/// @brief Structure to represent a semantic segmentation layer of the base pointcloud
using BaseSemanticSegmentationLayer =
  common::types::point_cloud::BasePointCloud<point::SemanticSegmentationData>;

/// @brief Structure to represent a semantic segmentation layer of the unstructured pointcloud
using UnstructuredSemanticSegmentationLayer =
  common::types::point_cloud::UnstructuredPointCloud<point::SemanticSegmentationData>;

/// @brief Structure to represent a semantic segmentation layer of the structured pointcloud
using StructuredSemanticSegmentationLayer =
  common::types::point_cloud::StructuredPointCloud<point::SemanticSegmentationData>;

} // namespace point_cloud
} // namespace types
} // namespace semantic_segmentation
} // namespace lum

#endif
