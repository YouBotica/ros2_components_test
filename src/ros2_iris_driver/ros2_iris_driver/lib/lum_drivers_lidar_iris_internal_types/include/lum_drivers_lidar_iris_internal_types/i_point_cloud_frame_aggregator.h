// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_POINT_CLOUD_FRAME_AGGREGATOR_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_POINT_CLOUD_FRAME_AGGREGATOR_H

#include <memory>

#include <lum_drivers_lidar_iris_internal_types/data_types.h>
#include <lum_drivers_lidar_iris_types/point_cloud_layer.h>

namespace lum {
namespace drivers {
namespace lidar {

namespace calibration {
struct CalibrationParameters;
}

namespace iris {

/// @brief Class that takes Unstructured PCL Lines, and emits Unstructured Point Clouds composed of
/// multiple lines
///
/// @note data emitted by this class has a short lifetime, so copy any data for use outside of the
/// subscription callback
///
/// @note This class uses the following calibration parameters:
/// 1) lidar_return - controls which points will be filtered out. For possible options please refer
/// to MultiReturnFilter
/// 2,3) min_depth and max_depth - control min amd max range for valid points. All points outside of
/// this range will be marked as invalid.
///
/// @note This class can filter out points depending on lidar_return calibration parameter
/// @code
/// using namespace lidar = lum::drivers::lidar;
/// // default instance, no filtering, split on scan count only
/// lidar::iris::calibration::CalibrationParameters config;
/// auto aggregator = lidar::iris::makePointCloudFrameAggregator(config);
///
/// // get point cloud line from a scan line processor
/// // or create a line as input
/// lidar::iris::types::point_cloud::UnstructuredLayeredData line{};
/// // populate line with points
/// // ...
///
/// // subscribe to frame events
/// auto sub = aggregator->subscribeOnFrameEnd([](const auto& frame) {
///   // act on frame data
///   // copy the data before returning if you want to reference it later
/// });
///
/// // add the line to the point cloud
/// aggregator->addLine(line);
///
/// @endcode
class IPointCloudFrameAggregator
{
public:
  // Polymorphic base class boilerplate
  virtual ~IPointCloudFrameAggregator() = default;
  IPointCloudFrameAggregator() = default;
  IPointCloudFrameAggregator(const IPointCloudFrameAggregator&) = delete;
  IPointCloudFrameAggregator(IPointCloudFrameAggregator&&) = delete;
  IPointCloudFrameAggregator& operator=(const IPointCloudFrameAggregator&) & = delete;
  IPointCloudFrameAggregator& operator=(IPointCloudFrameAggregator&&) & = delete;

  /// @brief update the calibration of this system
  /// @note changes to number of eyes is ignored
  ///@param [in] calibration new calibration
  virtual void updateCalibration(const calibration::CalibrationParameters& calibration) = 0;

  /// @brief get the current calibration of this system
  /// @return the current calibration
  virtual calibration::CalibrationParameters getCalibration() const = 0;

  /// @brief reset messages and current time
  virtual void reset() = 0;

  /// @brief add a line of data to the aggregator
  /// @param [in] line the line of data
  virtual void addLine(const types::point_cloud::UnstructuredLayeredData& line) = 0;

  /// @brief subscribe to access new frame data.
  /// @param [in] callback function to handle new frame data
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::UnstructuredLayeredDataSubscription
  subscribeOnFrameEnd(const types::UnstructuredLayeredDataSubscriptionCallback& callback) = 0;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
