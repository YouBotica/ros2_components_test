// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_A_LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_A_H
#define LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_A_LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_A_H

#include <cstdint>
#include <memory>

#include <lum_drivers_lidar_iris_internal_types/i_scan_line_processor.h>
#include <lum_drivers_lidar_iris_types/data_types.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief make an instance of the scan line processor A variant
/// @param [in] max_line_size the upper bound for the number of points in a line
/// @returns an instance of the processor
std::unique_ptr<IScanLineProcessor>
makeScanLineProcessorA(std::size_t max_line_size = DEFAULT_MAX_LINE_SIZE);

/// @brief make an instance of the legacy scan line processor A variant
/// @param [in] sensor_id the id associated with the sensor data to be processed
/// @returns an instance of the processor
std::unique_ptr<IScanLineProcessor> makeScanLineProcessorALegacy(std::uint8_t sensor_id);
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_A_LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_A_H
