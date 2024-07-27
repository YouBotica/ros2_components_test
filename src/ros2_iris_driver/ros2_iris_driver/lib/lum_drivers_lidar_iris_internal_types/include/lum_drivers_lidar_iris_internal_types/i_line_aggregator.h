// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_LINE_AGGREGATOR_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_LINE_AGGREGATOR_H

#include <lum_drivers_lidar_iris_internal_types/data_types.h>
#include <lum_drivers_lidar_iris_types/point_cloud_layer.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class ILineAggregator
{
public:
  /// @brief reset the internal state of this processor
  virtual void reset() = 0;

  /// @brief add a collection of points to the line aggregator
  /// @param [in] points the collection of points
  virtual void addPoints(const types::point_cloud::UnstructuredLayeredData& points) = 0;

  /// @brief subscribe to new line data
  /// @param [in] callback function to handle new line data
  /// @returns subscription handle, subscription is terminated when the handle goes out of scope
  /// @note  Data is temporary, and must be used within the callback, or copied for any later
  /// reference
  virtual types::UnstructuredLayeredDataSubscription subscribeOnScanLineSegment(
    const types::UnstructuredLayeredDataSubscriptionCallback& callback) = 0;

  // Boilerplate for an interface
  ILineAggregator() = default;
  virtual ~ILineAggregator() = default;
  ILineAggregator(const ILineAggregator&) = delete;
  ILineAggregator(ILineAggregator&&) = delete;
  ILineAggregator& operator=(const ILineAggregator&) & = delete;
  ILineAggregator& operator=(ILineAggregator&&) & = delete;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
