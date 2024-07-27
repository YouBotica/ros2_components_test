// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_TYPES_DATA_TYPES_H
#define LUM_DRIVERS_LIDAR_IRIS_TYPES_DATA_TYPES_H

#include <cstdint>
#include <memory>
#include <vector>

#include <lum_common_types_pointcloud/unstructured_point_cloud.h>
#include <lum_drivers_lidar_legacy_types/point.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief the number of detectors within the sensor we will see data from
constexpr std::uint8_t NUM_DETECTORS{2U};
/// @brief expected maximum returns that can be from a single emitted ray
constexpr std::uint8_t MAX_RETURNS_PER_RAY{6U};
/// @brief the number of rays we expect to see from the sensor per scan line
constexpr std::uint16_t NUM_RAYS_PER_LINE{1000U};
/// @brief the default maximum line size
constexpr std::size_t DEFAULT_MAX_LINE_SIZE{MAX_RETURNS_PER_RAY * NUM_RAYS_PER_LINE};

constexpr std::uint16_t POINT_CLOUD_A_SRC_PORT{4371U};  //< Source port for point cloud variant A
constexpr std::uint16_t POINT_CLOUD_B_SRC_PORT{46513U}; //< Source port for point cloud variant B

namespace types {

/// @brief indicates the quality of lidar data
enum class DataQualifier : std::uint8_t
{
  DQ_NORMAL,              ///< Sensor is operating normally, i.e. factory condition
  DQ_NOT_AVAILABLE,       ///< Data is not available, e.g. sensor in standby or thermal shutdown
  DQ_REDUCED_COVERAGE,    ///< Field of view is restricted, e.g. dirty window or configured FOV
  DQ_REDUCED_PERFORMANCE, ///< Sensor performance is reduced, e.g. laser or detector fault,
                          ///< thermal limits, bandwidth limits
  DQ_REDUCED_COVERAGE_AND_PERFORMANCE, ///< Field of view is restricted and sensor performance
                                       ///< is reduced, e.g dirty window and a laser fault
  DQ_TEST_MODE, ///< Sensor is running in a test mode; output differs from normal operation
  DQ_INVALID,   ///< Sensor configuration is invalid
};

/// @brief Detector site ID
enum class DetectorSiteId : std::uint8_t
{
  SITE_A = 0U,      ///< Site A
  SITE_B = 1U,      ///< Site B
  MAX_SITE = SITE_B ///< For error checking
};

struct SupplementalPointField
{
  std::uint8_t confidence_pct{}; ///< likelihood that the return is from a real world object, i.e.
                                 ///< not noise; valid range: 0-255
  std::uint8_t reserved{};       ///< second 8 bytes of debug/internal data
};

using SupplementalPointFields = std::vector<SupplementalPointField>;

struct LinePointsWithSupplementalFields
{
  using UnstructuredPointCloud =
    common::types::point_cloud::UnstructuredPointCloud<legacy::types::LinePoint>;

  using UnstructuredPointCloudPtr =
    common::types::point_cloud::UnstructuredPointCloudPtr<legacy::types::LinePoint>;

  UnstructuredPointCloudPtr points{std::make_shared<UnstructuredPointCloud>()};
  legacy::types::PointCloudLineMeta metadata{};
  SupplementalPointFields supplemental_fields{};
};

} // namespace types
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
