// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_FRAME_BOUNDARY_DETECTOR_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_FRAME_BOUNDARY_DETECTOR_H

#include <lum_drivers_lidar_iris_internal_types/data_types.h>
#include <lum_drivers_lidar_iris_types/point_cloud_layer.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// Keeps the state of the frame in order to determine if a new frame has occurred
class IFrameBoundaryDetector
{
public:
  /// Determines if a given packet @a metadata indicates a different frame compared to the last
  /// check and saves this value for the future
  /// @return Whether it's a different frame
  virtual bool checkNewFrame(const types::LidarPacketMetadata& metadata) = 0;

  /// Determines if a given packet @a lidar_data indicates a different frame compared to the last
  /// check and saves this value for the future
  /// @return Whether it's a different frame
  virtual bool checkNewFrame(const types::point::LidarData& lidar_data) = 0;

  // Polymorphic base class boilerplate
  virtual ~IFrameBoundaryDetector() = default;
  IFrameBoundaryDetector() = default;
  IFrameBoundaryDetector(const IFrameBoundaryDetector&) = delete;
  IFrameBoundaryDetector(IFrameBoundaryDetector&&) = delete;
  IFrameBoundaryDetector& operator=(const IFrameBoundaryDetector&) & = delete;
  IFrameBoundaryDetector& operator=(IFrameBoundaryDetector&&) & = delete;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
