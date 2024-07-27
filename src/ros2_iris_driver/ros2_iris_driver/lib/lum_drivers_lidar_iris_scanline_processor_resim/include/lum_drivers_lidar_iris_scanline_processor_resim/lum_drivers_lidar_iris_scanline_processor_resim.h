// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_RESIM_LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_RESIM_H
#define LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_RESIM_LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_RESIM_H

#include <cstdint>
#include <memory>

#include <lum_drivers_lidar_iris_types_resim/defaults.h>
#include <lum_drivers_lidar_iris_types_resim/i_resim_scanline_processor.h>
#include <lum_drivers_lidar_iris_types_resim/region_of_interest.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief Create an Iris ReSim processor for decompressing point clouds
/// @param sensor_id The sensor ID whose data should be monitored
/// @param resim_cal_path The path to the calibration file
/// @param batch_size the number of packets to be processed at once (default 1)
/// @param roi The region of interest for the resim processor use in filtering incoming data
/// @return The configured resim processor, or null if configuration failed
std::unique_ptr<IResimScanLineProcessor>
makeSyncScanLineProcessorResim(std::uint8_t sensor_id,
                               const std::string& resim_cal_path,
                               std::size_t batch_size = resim::DEFAULT_BATCH_SIZE,
                               const resim::IrisRegionOfInterest& roi = {});

/// @brief Create an Iris ReSim processor for decompressing point clouds
/// @param sensor_id The sensor ID whose data should be monitored
/// @param resim_cal_path The path to the calibration file
/// @param num_threads how parallel to run in the asynchronous client
/// @param batch_size the number of packets to be processed at once
/// @param internal_queue_size the number of batches that may be queued across threads
/// @param roi The region of interest for the resim processor use in filtering incoming data
/// @return The configured resim processor, or null if configuration failed
std::unique_ptr<IResimScanLineProcessor>
makeAsyncScanLineProcessorResim(std::uint8_t sensor_id,
                                const std::string& resim_cal_path,
                                std::size_t num_threads,
                                std::size_t batch_size,
                                std::size_t internal_queue_size,
                                const resim::IrisRegionOfInterest& roi = {});

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_RESIM_LUM_DRIVERS_LIDAR_IRIS_SCANLINE_PROCESSOR_RESIM_H
