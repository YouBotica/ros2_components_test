// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_POINT_CLOUD_PACKET_PARSER_H
#define LUM_DRIVERS_LIDAR_IRIS_INTERNAL_TYPES_I_POINT_CLOUD_PACKET_PARSER_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include <lum_drivers_lidar_iris_internal_types/data_types.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

/// @brief Converts lidar data packets into a bundle of rays w/ packet metadata
class IPointCloudPacketParser
{
public:
  /// @brief Packet parsing result codes
  enum class ParsingStatus : std::uint32_t
  {
    OK = 0U,
    WRONG_SEQUENCE = 1U,
    EXCESS_RETURNS = 2U,
    EXCESS_BYTES = 3U,
    INCOMPLETE_PACKET = 4U,
    BAD_PACKET = 5U,
  };

  /// Shorthand for what the parse function returns
  using ParseResult = std::tuple<ParsingStatus, types::LidarPacketMetadata, types::LidarRays>;

  /// @brief Parses the given lidar data packet and outputs a bundle of rays w/ packet metadata
  /// @param[in] packet_buffer the packet buffer
  /// @param[in] packet_size the packet size (in bytes)
  /// @return the parsing result code with the metadata and rays
  virtual ParseResult parsePacket(const std::uint8_t* packet_buffer, std::size_t packet_size) = 0;

  /// @brief Returns this processor's human-readable name and semver
  virtual std::pair<std::string, std::string> getNameAndVersion() const = 0;

  // Polymorphic base class boilerplate
  IPointCloudPacketParser() = default;
  virtual ~IPointCloudPacketParser() = default;
  IPointCloudPacketParser(const IPointCloudPacketParser&) = delete;
  IPointCloudPacketParser(IPointCloudPacketParser&&) = delete;
  IPointCloudPacketParser& operator=(const IPointCloudPacketParser&) & = delete;
  IPointCloudPacketParser& operator=(IPointCloudPacketParser&&) & = delete;
};

/// Gets a string corresponding to the enum values of IPointCloudPacketParser::ParsingStatus
constexpr const char* getParsingStatusMessage(const IPointCloudPacketParser::ParsingStatus status)
{
  constexpr std::array<const char*, 6> PARSING_RESULT_MSG{{
    "Packet is successfully parsed",
    "Wrong sequence of packets - some information might be dropped",
    "Packet contains one or more rays with more returns than what is supported",
    "Packet contains more bytes than can be parsed",
    "Packet contains fewer bytes than expected",
    "Packet cannot be parsed",
  }};
  const auto index = static_cast<std::size_t>(status);
  assert(index < PARSING_RESULT_MSG.size());
  return PARSING_RESULT_MSG[index];
}

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif
