// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_DRIVERS_PCAP_TYPES_H
#define LUM_DRIVERS_PCAP_TYPES_H

#include <lum_common_casts/lum_casts.h>
#include <lum_common_types/utils.h>
#include <lum_common_types_internal/networking.h>

namespace lum {
namespace drivers {
namespace pcap {
namespace types {

constexpr std::size_t MAX_UDP_DATA_SIZE = lum::common::types::networking::DEFAULT_UDP_PACKET_SIZE -
                                          lum::common::types::networking::MIN_UDP_DATAGRAM_SIZE;

PACKED_DATA(struct Ipv4Packet {
  lum::common::types::networking::EthernetHeader ethernet_header{};
  lum::common::types::networking::IpHeader ip_header{};
  lum::common::types::networking::UdpHeader udp_header{};
  std::array<std::uint8_t, MAX_UDP_DATA_SIZE> udp_data{};
});

} // namespace types
} // namespace pcap
} // namespace drivers
} // namespace lum

#endif // LUM_DRIVERS_PCAP_TYPES_H
