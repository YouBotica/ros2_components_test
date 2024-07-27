// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_SEMANTIC_SEGMENTATION_INTERNAL_TYPES_TYPES_H
#define LUM_SEMANTIC_SEGMENTATION_INTERNAL_TYPES_TYPES_H

#include <array>
#include <cstdint>
#include <limits>

#include <lum_common_types_classification/ontology.h>
#include <lum_common_types_point/point.h>
#include <lum_common_types_pointcloud/structured_point_cloud.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>
#include <lum_semantic_segmentation_types/types.h>

#ifndef CUDA_HOST_DEVICE
#if defined __CUDACC__
#define CUDA_HOST_DEVICE __host__ __device__
#else
#define CUDA_HOST_DEVICE
#endif
#endif

namespace lum {
namespace semantic_segmentation {
namespace internal_types {

/// @brief Structure to represent 3D points with classification
struct alignas(16) ClassifiedPoint : public common::types::point_cloud::Point
{
  float x{0.0}; ///< Coordinate x in cartesian space (in meters)
  float y{0.0}; ///< Coordinate y in cartesian space (in meters)
  float z{0.0}; ///< Coordinate z in cartesian space (in meters)

  /// @brief Return intensity. Intensity is a measure of optical power you get
  /// back from a target. Intensity values are between 0 and 4095:
  ////// 0 if no photons were returned
  ////// 2047 if photons are returned from perfect flat matte white body known as Lambertian target
  /// (94% reflective)
  ////// 2048 - 4095 shiny or specular object e.g. mirror or retro-reflector
  std::uint16_t intensity{0};

  /// @brief Azimuth coordinate in spherical space (in radians)
  float azimuth{0.0};
  /// @brief Elevation coordinate in spherical space; (in radians)
  float elevation{0.0};
  /// @brief Depth coordinate in spherical space (in meters)
  float depth{0.0};

  /// @brief nth index of last return, i.e. 0-3.
  /// Each laser pulse that is sent out can encounter objects in its path and
  /// record up to 3 returns or objects that it encounters
  std::uint8_t last_return_index{0};

  static constexpr std::uint32_t UNKNOWN_INSTANCE_ID{std::numeric_limits<std::uint32_t>::max()};

  ///< ID of the cluster to which this point belongs
  std::uint32_t instance_id{UNKNOWN_INSTANCE_ID};

  /// @brief Index used to pass information precomputed in cuda kernel to CPU.
  // Represents index from sparse back into dense point cloud.
  std::uint32_t hw_index{UNKNOWN_INSTANCE_ID};

  double timestamp{0.0}; ///< Timestamp (in seconds.microseconds)

  std::uint8_t track_number{0}; ///< Track number

  common::types::geometry::Vector2f velocity{0.0F, 0.0F}; ///< Velocity vector

  /// @brief Array of classifications, in order from most to least confident, i.e. entry 0 will be
  /// the top classification, entry 1 will be the second best classification
  lum::common::types::classification::ClassificationWithConfidence
    classifications[types::NUM_POINT_CLASSIFICATIONS];

  /// @brief Sensor ID this point was originated from (in multi-sensor setup)
  std::uint8_t sensor_id{0};

  std::uint8_t reserved{0}; ///< Reserved data
};

/// @brief returns true if this point is not valid e.g. is not set or received
/// no return from the LiDAR
CUDA_HOST_DEVICE constexpr bool isInvalid(const ClassifiedPoint& point)
{
  return point.x == 0.0F && point.y == 0.0F && point.z == 0.0F && point.intensity == 0;
}

/// @brief returns true if this point is a retroreflector
CUDA_HOST_DEVICE constexpr bool isRetroReflector(const ClassifiedPoint& point)
{
  return point.intensity >= types::MIN_RETRO_REFLECTOR_INTENSITY;
}

/// @brief returns true if this point has not been clustered
CUDA_HOST_DEVICE constexpr bool isClustered(const ClassifiedPoint& point)
{
  return point.instance_id !=
         lum::semantic_segmentation::internal_types::ClassifiedPoint::UNKNOWN_INSTANCE_ID;
}

/// @brief Enumeration of possible lane colors
enum class LaneColor : std::uint8_t
{
  NONE = 0,   ///< Sentinel: no lane color
  YELLOW = 1, ///< yellow lane
  WHITE = 2   ///< white lane
};

/// @brief Enumeration of cardinal and intercardinal directions
enum class HeadingDirection : std::uint8_t
{
  NONE = 0,       ///< Sentinel: no direction supplied
  NORTH = 1,      ///< north direction
  NORTH_EAST = 2, ///< north east direction
  EAST = 3,       ///< east direction
  SOUTH_EAST = 4, ///< south east direction
  SOUTH = 5,      ///< south direction
  SOUTH_WEST = 6, ///< south west direction
  WEST = 7,       ///< west direction
  NORTH_WEST = 8  ///< north west direction
};

/// @brief Structure for detailing a point classified as lane.
/// @note This point may have been directly measured in the pointcloud or interpolated based on
/// neighboring context
struct alignas(16) ClassifiedLanePoint : public common::types::point_cloud::Point
{
  float x{0.0}; ///< Coordinate x in cartesian space (in meters)
  float y{0.0}; ///< Coordinate y in cartesian space (in meters)
  float z{0.0}; ///< Coordinate z in cartesian space (in meters)

  /// @brief Lane boundary classification. A value of NONE indicates that this lane point lies
  /// directly on the lane. A value of EAST implies that this lane boundary points to a lane
  /// directly to the right. A value of WEST implies that this lane boundary points to a lane
  /// directly to the left.
  std::uint8_t boundary{static_cast<std::uint8_t>(HeadingDirection::NONE)};

  /// @brief Lane color classification. A value of NONE indicates that the color of this lane point
  /// could not be classified. A value of YELLOW indicates a yellow lane, a value of WHITE indicates
  /// a white lane.
  std::uint8_t color{static_cast<std::uint8_t>(LaneColor::NONE)};

  static constexpr std::uint32_t UNKNOWN_INSTANCE_ID{std::numeric_limits<std::uint32_t>::max()};

  /// @brief Lane instance id. This id is unique for every cluster of lane points.
  std::uint32_t instance_id{UNKNOWN_INSTANCE_ID};

  /// @brief Index used to pass information precomputed in cuda kernel to CPU.
  // Represents index from sparse back into dense point cloud.
  std::uint32_t hw_index{UNKNOWN_INSTANCE_ID};

  /// @brief Lane classification with confidence. Solid, dashed, etc. See ontology for full
  /// definition
  lum::common::types::classification::ClassificationWithConfidence classification;
};

/// @brief Structure for detailing a point classified as road or road edge
/// @note This point may have been directly measured in the pointcloud or interpolated based on
/// neighboring context
struct alignas(16) ClassifiedRoadPoint : public common::types::point_cloud::Point
{
  float x{0.0}; ///< Coordinate x in cartesian space (in meters)
  float y{0.0}; ///< Coordinate y in cartesian space (in meters)
  float z{0.0}; ///< Coordinate z in cartesian space (in meters)

  /// @brief Road edge boundary classification. A value of NONE indicates that this road edge point
  /// point lies directly on the estimated road surface. A value of EAST implies that this road edge
  /// boundary points to road surface to the right. A value of WEST implies that this road edge
  /// boundary points to road surface to the left.
  std::uint8_t boundary{static_cast<std::uint8_t>(HeadingDirection::NONE)};

  /// @brief Road/road edge classification with confidence. See ontology for
  /// full definition
  lum::common::types::classification::ClassificationWithConfidence classification;
};

/// @brief Full output of semantic segmentation model
struct ModelOutput
{
  /// @brief Structured point cloud with road, lane, and object classifications represented on the
  /// original input point cloud
  common::types::point_cloud::StructuredPointCloudPtr<ClassifiedPoint> classified_cloud;

  /// @brief Unstructured point cloud with classified lane/lane boundary points
  common::types::point_cloud::UnstructuredPointCloudPtr<ClassifiedLanePoint> lane_cloud;

  /// @brief Unstructured point cloud with classified road/road edge points
  common::types::point_cloud::UnstructuredPointCloudPtr<ClassifiedRoadPoint> road_cloud;

  /// @brief Vector of bounding boxes with top-3 classifications and confidences
  std::vector<types::ClassifiedBoundingBox3D> objects;

  /// @brief Vector of barriers represented by two surrounding polylines and classification
  std::vector<types::Barrier> barriers;
};

} // namespace internal_types
} // namespace semantic_segmentation
} // namespace lum

#endif
