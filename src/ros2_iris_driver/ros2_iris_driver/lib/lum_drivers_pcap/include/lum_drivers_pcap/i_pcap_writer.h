// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_PCAP_I_PCAP_WRITER_H
#define LUM_DRIVERS_PCAP_I_PCAP_WRITER_H

#include <chrono>

#include <lum_drivers_pcap/types.h>

namespace lum {
namespace drivers {
namespace pcap {

/// Encapsulates PCAP writing
class IPcapWriter
{
public:
  /// Writes a single packet to disk
  /// @param packet the packet to save to disk
  /// @param timestamp the timestamp of the packet in microseconds
  /// @return True on success, false on failure.
  virtual bool addPacket(const types::Ipv4Packet& packet,
                         const std::chrono::microseconds& timestamp) = 0;

  /// Close the file(s) for writing and flush everything to disk.
  /// @return True if the file closed successfully, false on failure.
  virtual bool closeWriter() = 0;

  // Polymorphic base class boilerplate
  virtual ~IPcapWriter() = default;
  IPcapWriter() = default;
  IPcapWriter(const IPcapWriter&) = delete;
  IPcapWriter(IPcapWriter&&) = delete;
  void operator=(const IPcapWriter&) & = delete;
  void operator=(IPcapWriter&&) & = delete;
};

} // namespace pcap
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_PCAP_I_PCAP_WRITER_H
