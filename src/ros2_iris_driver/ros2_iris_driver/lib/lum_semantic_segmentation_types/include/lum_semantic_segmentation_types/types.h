// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_SEMANTIC_SEGMENTATION_TYPES_TYPES_H
#define LUM_SEMANTIC_SEGMENTATION_TYPES_TYPES_H

#include <array>
#include <cstdint>
#include <limits>

#include <lum_common_types/common.h>
#include <lum_common_types/geometry.h>
#include <lum_common_types_classification/ontology.h>
#include <lum_semantic_segmentation_types/point_cloud_layer.h>

namespace lum {
namespace semantic_segmentation {
namespace types {

static constexpr std::size_t NUM_POINT_CLASSIFICATIONS{3};
static constexpr std::size_t NUM_BOX_CLASSIFICATIONS{3};
static constexpr std::uint16_t MIN_RETRO_REFLECTOR_INTENSITY{2048};

/// @brief Bounding Box Structure for representing 3D bounding boxes
struct BoundingBox
{
  lum::common::types::geometry::Posef
    pose{}; ///< Pose of the frame attached to the center of the box
  lum::common::types::geometry::Vector3f scale{}; ///< Dimension of the box along x, y, z
};

/// @brief Bounding Box Structure for representing 3D bounding boxes with
/// classification
/// @tparam NumBoxClassifications number of classifications
template <std::size_t NumBoxClassifications>
struct BoundingBoxWithClassification : public BoundingBox
{
  static const std::size_t num_classifications = NumBoxClassifications;
  std::array<lum::common::types::classification::ClassificationWithConfidence,
             NumBoxClassifications>
    classifications{};
  std::array<common::types::geometry::Vector2f, 4U> box_corners;
  std::uint32_t instance_id{0U};        ///< object instance id
  std::uint32_t sensor_id{0U};          ///< object sensor id
  lum::common::types::Time timestamp{}; ///< Timestamp
};

/// @brief Bounding Box Structure for representing 3D bounding boxes with
/// NUM_BOX_CLASSIFICATIONS classifications
struct ClassifiedBoundingBox3D : public BoundingBoxWithClassification<NUM_BOX_CLASSIFICATIONS>
{
};

/// @brief Struct to represent barriers
struct Barrier
{
  lum::common::types::classification::OntologyType classification{};
  std::vector<lum::common::types::geometry::Vector2f>
    left_polyline{}; ///< vector to represent left boundary of barrier
  std::vector<lum::common::types::geometry::Vector2f>
    right_polyline{}; ///< vector to represent right boundary of barrier
  float max_height{std::numeric_limits<float>::lowest()};
  std::uint32_t instance_id{0U}; ///< object instance id
  std::uint32_t sensor_id{0U};   ///< object sensor id
};

/// @brief Full output of semantic segmentation model
struct ModelOutput
{
  /// @brief Structured point cloud with road, lane, and object classifications represented on the
  /// original input point cloud
  point_cloud::StructuredSemanticSegmentationLayer classified_cloud;

  /// @brief Vector of bounding boxes with top-3 classifications and confidences
  std::vector<ClassifiedBoundingBox3D> objects;

  /// @brief Vector of barriers represented by two surrounding polylines and classification
  std::vector<Barrier> barriers;
};

} // namespace types
} // namespace semantic_segmentation
} // namespace lum

#endif
