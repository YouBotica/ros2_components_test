// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_FRAME_BOUNDARY_DETECTOR_FRAME_BOUNDARY_DETECTOR_H
#define LUM_DRIVERS_LIDAR_IRIS_FRAME_BOUNDARY_DETECTOR_FRAME_BOUNDARY_DETECTOR_H

#include <memory>

#include <lum_drivers_lidar_iris_internal_types/i_frame_boundary_detector.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// Returns an instantiation of the IFrameBoundaryDetector
std::unique_ptr<IFrameBoundaryDetector> makeFrameBoundaryDetector();

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
