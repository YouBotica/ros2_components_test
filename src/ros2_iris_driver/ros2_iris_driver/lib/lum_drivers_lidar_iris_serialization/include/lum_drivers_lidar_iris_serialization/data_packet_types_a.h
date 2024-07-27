// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_SERIALIZATION_DATA_PACKET_TYPES_A_H
#define LUM_DRIVERS_LIDAR_IRIS_SERIALIZATION_DATA_PACKET_TYPES_A_H

#include <cstdint>

#include <lum_common_types_internal/macros.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {
namespace serialization {

// Iris point cloud types (wire-format)
// See data spec:
// https://github.com/luminartech/iris_firmware/blob/master/architecture/data_spec_v1.md

constexpr float ANGLE_CONVERSION_RADIANS_TO_FIXED{16384.0F}; // converts from radians to Q2.14
constexpr float RANGE_CONVERSION_METERS_TO_FIXED{4096.0F};   // converts meters to UQ12.12
constexpr float REFLECTANCE_CONVERSION_FACTOR{32768.0F};     // converts reflectance to UQ1.15

/// @brief Lidar data protection footer size (in bytes)
constexpr std::size_t DATA_PROTECTION_FOOTER_SIZE{24U};

/// @brief Lidar packet header
PACKED_DATA(struct LidarPacketHeader {
  std::uint8_t packet_version_major;
  std::uint8_t packet_version_minor;
  std::uint8_t packet_version_patch;
  std::uint8_t packet_sequence;
  std::uint8_t num_rays;
  std::uint8_t frame_sequence;
  std::uint64_t ptp_timestamp_seconds : 48;
  std::uint64_t sensor_id : 8;
  std::uint64_t data_qualifier : 8;
  std::uint16_t reserved;
});

/// @brief Lidar ray header
PACKED_DATA(struct LidarRayHeader {
  std::int16_t azimuth_radians;
  std::int16_t elevation_radians;
  std::uint32_t ptp_timestamp_ns;
  std::uint8_t scan_checkpoint;
  std::uint8_t ray_sequence : 4;
  std::uint8_t num_returns : 4;
  std::uint16_t detector_number : 1;
  std::uint16_t blockage_level : 4;
  std::uint16_t reserved_0 : 2;
  std::uint16_t line_number : 9;
  std::uint32_t reserved_1;
});

/// @brief Lidar ray return
PACKED_DATA(struct LidarReturn {
  std::uint32_t existence_probability_pct : 8;
  std::uint32_t range_meters : 24;
  std::uint16_t reflectance;
  std::uint8_t reserved_envision;
  std::uint8_t reserved;
});

// Assert that these structs are the expected size.
static_assert(sizeof(LidarPacketHeader) == 16, "LidarPacketHeader is not the expected size");
static_assert(sizeof(LidarRayHeader) == 16, "LidarRayHeader is not the expected size");
static_assert(sizeof(LidarReturn) == 8, "LidarReturn is not the expected size");

} // namespace serialization
} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
