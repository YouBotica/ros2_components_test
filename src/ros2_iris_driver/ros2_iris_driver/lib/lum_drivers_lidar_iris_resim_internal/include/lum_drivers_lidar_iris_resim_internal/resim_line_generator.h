// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_RESIM_LINE_GENERATOR_H
#define LUM_DRIVERS_LIDAR_IRIS_RESIM_INTERNAL_RESIM_LINE_GENERATOR_H

#include <memory>

#include <lum_drivers_lidar_iris_internal_types/data_types.h>
#include <lum_drivers_lidar_iris_line_generator_a/i_point_cloud_line_generator_a.h>
#include <schemas/pointcloud.capnp.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class ResimScanLineGenerator
{
public:
  explicit ResimScanLineGenerator(std::unique_ptr<IPointCloudLineGeneratorA> line_generator);

  std::uint8_t getSensorId() const { return line_generator_->getSensorId(); }

  void setSensorId(std::uint8_t sensor_id) { line_generator_->setSensorId(sensor_id); }

  void reset() { line_generator_->reset(); }

  bool addPointCloudMessage(const ReSim::Schemas::PointCloud::Reader& msg);

  iris::types::PointCloudLineSubscription
  subscribeOnScanLineSegment(const iris::types::PointCloudLineSubscriptionCallback& callback);

  iris::types::PointCloudSupplementalLineSubscription subscribeOnScanLineSegment(
    const iris::types::PointCloudSupplementalLineSubscriptionCallback& callback);

private:
  std::unique_ptr<IPointCloudLineGeneratorA> line_generator_;
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
