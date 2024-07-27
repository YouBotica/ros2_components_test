// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_CONFIG_H
#define LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_CONFIG_H

#include <cstddef>
#include <string>

#include <lum_drivers_lidar_iris_types_resim/defaults.h>
#include <lum_drivers_lidar_iris_types_resim/region_of_interest.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace resim {

/// @brief The configuration options used by both the sync and async ReSim processors
struct ResimConfig
{
  std::string cal_path{};                            //< ReSim calibration file path
  std::size_t batch_size{resim::DEFAULT_BATCH_SIZE}; //< the size of each packet batch
  resim::IrisRegionOfInterest roi{}; //< acts as a lidar data filter; can reduce processing load
};

/// @brief The configuration options used by only the async ReSim processor
struct AsyncResimConfig
{
  std::size_t num_threads{DEFAULT_WORKER_COUNT};       //< number of worker threads
  std::size_t internal_queue_size{DEFAULT_QUEUE_SIZE}; //< maximum size of batch queue
};

} // namespace resim
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_CONFIG_H
