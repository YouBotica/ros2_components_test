// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_PORTS_H
#define LUM_DRIVERS_LIDAR_IRIS_TYPES_RESIM_PORTS_H

#include <cstdint>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace resim {

constexpr std::uint16_t DATA_SRC_PORT{4369U};   //< Source port for resim data
constexpr std::uint16_t STATUS_SRC_PORT{4370U}; //< Source port for resim status
constexpr std::uint16_t IRIS_DEST_PORT{4370U};  //< Destination port for resim packets

} // namespace resim
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
