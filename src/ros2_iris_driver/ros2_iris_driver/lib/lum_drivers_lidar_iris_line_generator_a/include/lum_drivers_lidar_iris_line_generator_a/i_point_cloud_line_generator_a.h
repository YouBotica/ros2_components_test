// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_LINE_GENERATOR_A_I_POINT_CLOUD_LINE_GENERATOR_A_H
#define LUM_DRIVERS_LIDAR_IRIS_LINE_GENERATOR_A_I_POINT_CLOUD_LINE_GENERATOR_A_H

#include <lum_drivers_lidar_iris_internal_types/i_point_cloud_line_generator.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class IPointCloudLineGeneratorA : public IPointCloudLineGenerator
{
public:
  /// @brief Add rays for point cloud generation, accumulating until a full cloud is received, then
  /// calls any callbacks and starts afresh.
  /// @param [in] metadata The data about the data
  /// @param [in] rays The data to add to the point cloud
  virtual void add(const types::LidarPacketMetadata& metadata, const types::LidarRays& rays) = 0;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_LINE_GENERATOR_A_I_POINT_CLOUD_LINE_GENERATOR_A_H
