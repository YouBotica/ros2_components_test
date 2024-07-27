// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_STAMPED_DATA_HEADER_H
#define LUM_COMMON_TYPES_STAMPED_DATA_HEADER_H

#include <array>
#include <cstdint>
#include <cstring>
#include <tuple>

#include <lum_common_types/common.h>

namespace lum {
namespace common {
namespace types {

/// @brief Structure to represent the metadata for high level stamped data types. This is used to
/// communicate timestamped data in particular coordinate frame
struct StampedDataHeader
{
  static constexpr std::uint8_t MAX_SIZE{255U};

  ///< Consecutively increasing Sequence ID
  std::uint32_t sequence_id{};

  ///< Timestamp for the data type
  Time timestamp{};

  ///< Coordinate frame from which the data originated
  std::array<char, StampedDataHeader::MAX_SIZE + 1U> coordinate_frame_name{};
};

inline void storeCoordinateFrameName(StampedDataHeader& header,
                                     const std::string& input_string) noexcept
{
  std::ignore =
    strncpy(header.coordinate_frame_name.data(), input_string.c_str(), StampedDataHeader::MAX_SIZE);
}

inline const char* getCoordinateFrameName(const StampedDataHeader& header) noexcept
{
  return header.coordinate_frame_name.data();
}

} // namespace types
} // namespace common
} // namespace lum

#endif
