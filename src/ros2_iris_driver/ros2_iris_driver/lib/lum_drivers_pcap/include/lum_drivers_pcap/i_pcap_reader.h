// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_PCAP_I_PCAP_READER_H
#define LUM_DRIVERS_PCAP_I_PCAP_READER_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <lum_common_types/networking.h>
#include <lum_common_types/result.h>

namespace lum {
namespace drivers {
namespace pcap {

/// Encapsulates PCAP reading
class IPcapReader
{
public:
  /// Status values (values <= -100 are drawn from the pcap_next_ex() docs)
  enum class Status : std::int32_t
  {
    SUCCESS = 1,
    TIMEOUT = 0,
    READ_ERROR = -1,
    END_OF_FILE = -2,
    UNDOCUMENTED_PCAP_ERROR = -100, ///< unexpected error not found in the PCAP docs
    PARSE_ERROR = -200,             ///< error encountered AFTER raw PCAP data was retrieved
    INSUFFICIENT_SIZE = -300        //< Packet buffer size is insufficient
  };

  /// The metadata related to a particular packet that is returned by a read operation.
  struct PacketMetadata
  {
    std::size_t packet_size{0U};
    std::chrono::microseconds timestamp{};
    common::types::networking::UdpHeader udp_header{};
    std::uint32_t source_ip_address{};
  };

  /// The result of a read operation, which can fail
  using Result = common::types::Result<PacketMetadata, Status>;

  /// Reads a single packet into the given buffer
  /// @param packet_payload the buffer to copy the packet payload into
  /// @param max_bytes_to_write the maximum number of bytes that could be written into @a
  /// packet_payload
  /// @return The status and metadata for the data packet. If status is success, the metadata is
  /// valid and @a packet_payload contains the data from the packet. If the status is not success,
  /// the metadata contents are undefined, and the contents of @a packet_payload should be
  /// considered invalid, though they may have still been modified.
  virtual Result readOnePacketInto(void* packet_payload, std::size_t max_bytes_to_write) = 0;

  /// Rewinds the file to the beginning
  /// @return whether the rewind succeeded
  virtual bool rewindToStart() = 0;

  // Polymorphic base class boilerplate
  virtual ~IPcapReader() = default;
  IPcapReader() = default;
  IPcapReader(const IPcapReader&) = delete;
  IPcapReader(IPcapReader&& other) = delete;
  void operator=(const IPcapReader&) & = delete;
  void operator=(IPcapReader&&) & = delete;
};

} // namespace pcap
} // namespace drivers
} // namespace lum

#endif
