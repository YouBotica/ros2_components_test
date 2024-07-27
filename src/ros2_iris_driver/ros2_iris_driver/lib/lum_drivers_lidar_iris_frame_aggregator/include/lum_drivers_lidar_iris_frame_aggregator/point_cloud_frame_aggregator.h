// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_FRAME_AGGREGATOR_POINT_CLOUD_FRAME_AGGREGATOR_H
#define LUM_DRIVERS_LIDAR_IRIS_FRAME_AGGREGATOR_POINT_CLOUD_FRAME_AGGREGATOR_H

#include <memory>

#include <lum_drivers_lidar_calibration/calibration.h>
#include <lum_drivers_lidar_iris_internal_types/i_point_cloud_frame_aggregator.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// Returns an instantiation of the IPointCloudFrameAggregator
/// @param calibration The calibration object to customize a point cloud processing pipeline
std::unique_ptr<IPointCloudFrameAggregator>
makePointCloudFrameAggregator(const calibration::CalibrationParameters& calibration = {});

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
