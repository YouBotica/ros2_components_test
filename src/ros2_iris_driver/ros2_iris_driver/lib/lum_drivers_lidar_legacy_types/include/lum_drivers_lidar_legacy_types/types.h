// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_LEGACY_TYPES_TYPES_H
#define LUM_DRIVERS_LIDAR_LEGACY_TYPES_TYPES_H

#include <cstdint>

namespace lum {
namespace drivers {
namespace lidar {
namespace legacy {
namespace types {

struct PacketPreamble
{
  std::uint8_t num_rays{};           ///< Number of rays contained in the data of this packet
  std::uint8_t sequence_number{};    ///< Sequence in lidar data packets from this head
  std::uint16_t fingerprint{};       ///< Fingerprint of the head sending this packet
  std::uint32_t timestamp_seconds{}; ///< 32-bit Unix timestamp (seconds)
  std::uint8_t scan_profile{};       ///< scan profile type in use during current frame
  std::uint8_t checkpoint{};         ///< what section of the current scan is in process
  std::uint8_t scan_counters{};      ///< index of current top-level scan, as reported by the sensor
  std::uint8_t interlace_index{};    ///< interlace index within the top-level scan, as reported
  std::uint8_t track_number{};       ///< playlist index, within the interlace index.
};

} // namespace types
} // namespace legacy
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
