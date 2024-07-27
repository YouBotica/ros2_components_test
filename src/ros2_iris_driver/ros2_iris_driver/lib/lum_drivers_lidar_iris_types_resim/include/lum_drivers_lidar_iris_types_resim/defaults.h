// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_DEFAULTS_H
#define LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_DEFAULTS_H

#include <cstdint>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace resim {

///////////////////////////////////////////////////////////////////////////////////////////////////
// memory tuning
// https://luminartech.atlassian.net/wiki/spaces/SWE/pages/1514308098/Async+Resim+Memory+Tuning
constexpr std::size_t DEFAULT_QUEUE_SIZE{2000U};
constexpr std::size_t DEFAULT_BATCH_SIZE{100U};
constexpr std::size_t DEFAULT_WORKER_COUNT{5U};

} // namespace resim
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_DEFAULTS_H
