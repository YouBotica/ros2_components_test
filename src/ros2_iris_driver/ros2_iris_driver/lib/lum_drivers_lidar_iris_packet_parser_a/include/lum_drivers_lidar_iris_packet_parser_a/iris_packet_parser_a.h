// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_LIDAR_IRIS_PACKET_PARSER_A_IRIS_PACKET_PARSER_A_H
#define LUM_DRIVERS_LIDAR_IRIS_PACKET_PARSER_A_IRIS_PACKET_PARSER_A_H

#include <array>
#include <cstdint>

#include <lum_drivers_lidar_iris_packet_parser_a/i_iris_packet_parser_a.h>

namespace lum {
namespace drivers {
namespace lidar {
namespace iris {

class IrisPacketParserA : public IIrisPacketParserA
{
public:
  ParseResult parsePacket(const std::uint8_t* packet_buffer, std::size_t packet_size) override;

  std::pair<std::string, std::string> getNameAndVersion() const override;

private:
  static constexpr std::array<std::uint8_t, 3U> PACKET_VERSION{1, 1, 0};

  /// @brief whether the next packet will be the first received valid packet
  bool first_valid_packet_{true};
  /// @brief sequence number of the last received packet
  std::uint8_t last_sequence_number_{0U};

  ParsingStatus validatePacketOrdering(const types::LidarPacketMetadata& metadata);
};

} // namespace iris
} // namespace lidar
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_LIDAR_IRIS_PACKET_PARSER_A_LUM_DRIVERS_LIDAR_IRIS_PACKET_PARSER_A_H
