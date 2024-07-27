// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_DATA_TYPES_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_DATA_TYPES_H

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <set>
#include <vector>

#include <lum_common_types/units.h>
#include <lum_common_types_observable/subscription.h>
#include <lum_common_types_pointcloud/structured_point_cloud.h>
#include <lum_common_types_pointcloud/unstructured_point_cloud.h>
#include <lum_drivers_lidar_iris_types/data_types.h>
#include <lum_drivers_lidar_iris_types/point_cloud_layer.h>
#include <lum_drivers_lidar_legacy_types/point.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace types {

/// @brief SemVer version struct
struct SemVer
{
  std::uint8_t major{};
  std::uint8_t minor{};
  std::uint8_t patch{};
};

/// @brief Lidar packet metadata
struct LidarPacketMetadata
{
  SemVer packet_version{}; ///< packet version

  std::uint8_t sensor_id{}; ///< configured sensor ID; default is 0.

  std::uint8_t sequence_number{}; ///< lidar packet sequence number for this sensor; increments
                                  ///< once per packet; wraps back to 0 when the max value is
                                  ///< reached; valid range: 0-255
  std::uint8_t
    frame_index{}; ///< lidar frame index for this sensor; increments for every new
                   ///< frame; wraps back to 0 when the max value is reached; valid range: 0-255

  std::uint8_t num_rays{}; ///< number of rays contained in the data of this packet

  DataQualifier data_qualifier{}; ///< self-assessment by the sensor of the validity of the lidar
                                  ///< data in this packet

  // Note: We can't use std::chrono::seconds because it is not guaranteed to hold the entire
  // 48-bit value here. Per the docs for std::chrono::seconds, it is only guaranteed to be a
  // signed integer type of at least 35 bits. Thus we define our own type that is guaranteed to
  // hold the incoming value and is measured in seconds.
  using Timestamp = std::chrono::duration<std::int64_t>;

  Timestamp timestamp{}; ///< the lidar system time rounded down to the nearest integer
                         ///< (in seconds) for all rays contained in the packet; uses
                         ///< PTP if available; otherwise, the value represents the
                         ///< amount of time since sensor bootup.
};

/// @brief represents the unique information from a single lidar ray return
struct LidarRayReturn
{
  static constexpr float MIN_RANGE_METERS = 0.0F;    ///< The range minimum for range in meters
  static constexpr float MAX_RANGE_METERS = 4096.0F; ///< The range maximum for range in meters

  float range_meters{}; ///< measured distance to the target from the lidar origin (in meters);
                        ///< valid range: 0-4096

  static constexpr float MIN_REFLECTANCE = 0.0F; ///< The range minimum for reflectance
  static constexpr float MAX_REFLECTANCE = 2.0F; ///< The range maximum for reflectance

  float reflectance{}; ///< estimated reflectance of the target; the value is the ratio between
                       ///< the quantity of optical energy reflected back to the receive
                       ///< aperture and the quantity that was estimated to have hit the target;
                       ///< 1 represents a 100% Lambertian target; valid range: 0-2

  static constexpr std::uint8_t MIN_CONFIDENCE_PCT = 0U;   ///< The range minimum for confidence
  static constexpr std::uint8_t MAX_CONFIDENCE_PCT = 255U; ///< The range maximum for confidence

  std::uint8_t confidence_pct{}; ///< likelihood that the return is from a real world object, i.e.
                                 ///< not noise; valid range: 0-255

  std::uint8_t reserved_envision{}; ///< first 8 bytes of debug/internal data
  std::uint8_t reserved{};          ///< second 8 bytes of debug/internal data
};

/// @brief a collection of returns the belong to a single lidar ray
using LidarRayReturns = std::array<LidarRayReturn, MAX_RETURNS_PER_RAY>;

/// @brief a struct holding the data for a single lidar ray, and its returns
struct LidarRay
{
  using Timestamp = std::chrono::nanoseconds;
  using Angle = common::types::units::AngleInRadians<float>;

  Timestamp timestamp{}; ///< the lidar system time when the ray was emitted (in
                         ///< nanoseconds); uses PTP if available; otherwise, the value
                         ///< represents the amount of time since sensor bootup.

  Angle azimuth{}; ///< azimuth value (in radians) of the emitted ray from
                   ///< the lidar's perspective where 0 is straight forward
                   ///< and positive angles are to the left

  Angle elevation{}; ///< elevation value (in radians) of the emitted ray from the lidar's
                     ///< perspective where 0 is straight forward and positive angles are up

  std::uint16_t line_index{}; ///< line index within the current frame; wraps back to 0 if the
                              ///< maximum value is reached; valid range: 0-511

  std::uint8_t scan_checkpoint{}; ///< most recent checkpoint value within the active scan
                                  ///< pattern; these values are user-defined within the scan
                                  ///< control interface; valid range: 0-255
  DetectorSiteId
    detector_site_id{}; ///< the detector site id for this lidar ray, i.e. Site A or Site B

  std::uint8_t blockage_level{}; ///< the detected blockage level for a given ray. valid range: 0-15

  std::uint8_t ray_index{}; ///< a monotonically increasing index for each ray; wraps back to 0 if
                            ///< the maximum value is reached; valid range 0-15

  std::uint8_t num_returns{}; ///< number of actual valid returns for this ray

  LidarRayReturns returns{}; ///< lidar ray returns
};

/// @brief type used to transfer multiple rays
using LidarRays = std::vector<LidarRay>;

/// @brief Iris line point type
using LinePoint = legacy::types::LinePoint;
/// @brief Iris point type
using Point = legacy::types::Point;
/// @brief Iris line metadata type
using PointCloudLineMeta = legacy::types::PointCloudLineMeta;

/// @brief Subscription for pointcloud of points with their line info
using PointCloudLineSubscription = common::types::observable::SubscriptionHandle<
  const common::types::point_cloud::UnstructuredPointCloudConstPtr<LinePoint>&,
  const PointCloudLineMeta&>;

/// @brief Subscription callback for pointcloud of points with their line info
using PointCloudLineSubscriptionCallback = std::function<void(
  const common::types::point_cloud::UnstructuredPointCloudConstPtr<LinePoint>& line,
  const PointCloudLineMeta& meta)>;

/// @brief subscription handle for point cloud lines with supplemental info
using PointCloudSupplementalLineSubscription =
  common::types::observable::SubscriptionHandle<const LinePointsWithSupplementalFields&>;

/// @brief subscription callback for point cloud lines with supplemental info
using PointCloudSupplementalLineSubscriptionCallback =
  std::function<void(const LinePointsWithSupplementalFields&)>;

/// @brief Layered data subscription type
using UnstructuredLayeredDataSubscription =
  common::types::observable::SubscriptionHandle<const point_cloud::UnstructuredLayeredData&>;

/// @brief Layered data subscription callback type
using UnstructuredLayeredDataSubscriptionCallback =
  std::function<void(const point_cloud::UnstructuredLayeredData&)>;

/// @brief Structured layered data subscription type
using StructuredLayeredDataSubscription =
  common::types::observable::SubscriptionHandle<const point_cloud::StructuredLayeredData&>;

/// @brief Layered data subscription callback type
using StructuredLayeredDataSubscriptionCallback =
  std::function<void(const point_cloud::StructuredLayeredData&)>;

/// @brief A set of 8-bit frame indices
using FrameIndices = std::set<std::uint8_t>;

} // namespace types
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
