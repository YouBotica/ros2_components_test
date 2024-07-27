// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_LINE_AGGREGATOR_LUM_DRIVERS_LIDAR_IRIS_LINE_AGGREGATOR_H
#define LUM_DRIVERS_LIDAR_IRIS_LINE_AGGREGATOR_LUM_DRIVERS_LIDAR_IRIS_LINE_AGGREGATOR_H

#include <cstdint>
#include <memory>

#include <lum_drivers_lidar_iris_internal_types/i_line_aggregator.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

std::unique_ptr<ILineAggregator> makeLineAggregator(std::size_t max_line_size);

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
