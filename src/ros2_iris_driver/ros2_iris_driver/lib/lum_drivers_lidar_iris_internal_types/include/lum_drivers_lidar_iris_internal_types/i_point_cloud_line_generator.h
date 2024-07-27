// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_POINT_CLOUD_LINE_GENERATOR_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_POINT_CLOUD_LINE_GENERATOR_H

#include <cstdint>

#include <lum_common_types_internal/tagged_bool.h>
#include <lum_drivers_lidar_iris_internal_types/data_types.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief Class used to generate unfiltered UnstructuredPointClouds that each contain a single scan
/// line.  The data output from this class is only valid during the callback; any consumer should
/// use it immediately or copy for later use.  The transmitted line segments are complete scan
/// lines.
class IPointCloudLineGenerator
{
public:
  using WithResimIndexing = common::types::TaggedBool<class WithResimIndexingTag>;

  /// @brief Get sensor id
  /// @return configured sensor id
  virtual std::uint8_t getSensorId() const = 0;

  /// @brief Set sensor id
  /// @param [in] sensor_id a new sensor_id to use
  virtual void setSensorId(std::uint8_t sensor_id) = 0;

  /// @brief reset the internal state of this processor
  virtual void reset() = 0;

  /// @brief subscribe to new line data
  /// @param [in] callback function to handle new line data
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::PointCloudLineSubscription
  subscribeOnScanLineSegment(const types::PointCloudLineSubscriptionCallback& callback) = 0;

  /// @brief subscribe to new line data
  /// @param [in] callback function to handle new line data with supplemental fields
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::PointCloudSupplementalLineSubscription subscribeOnScanLineSegment(
    const types::PointCloudSupplementalLineSubscriptionCallback& callback) = 0;

  // Boilerplate for an interface
  IPointCloudLineGenerator() = default;
  virtual ~IPointCloudLineGenerator() = default;
  IPointCloudLineGenerator(const IPointCloudLineGenerator&) = delete;
  IPointCloudLineGenerator(IPointCloudLineGenerator&&) = delete;
  IPointCloudLineGenerator& operator=(const IPointCloudLineGenerator&) & = delete;
  IPointCloudLineGenerator& operator=(IPointCloudLineGenerator&&) & = delete;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
