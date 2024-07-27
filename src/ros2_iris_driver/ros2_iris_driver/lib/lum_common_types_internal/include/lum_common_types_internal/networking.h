// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_INTERNAL_NETWORKING_H
#define LUM_COMMON_TYPES_INTERNAL_NETWORKING_H

#include <lum_common_types/networking.h>
#include <lum_common_types_internal/macros.h>

namespace lum {
namespace common {
namespace types {
namespace networking {

/// @brief Ethernet header size (not including vlan), in bytes
static constexpr std::size_t ETHERNET_HEADER_SIZE{14U};

/// @brief VLAN Tag size, in bytes
static constexpr std::size_t VLAN_TAG_SIZE{4U};

/// @brief Size of IPv4 header with no specified options, in bytes
static constexpr std::size_t MIN_IP_V4_HEADER_SIZE{20U};

/// @brief Minimum UDP packet size, in bytes
static constexpr std::size_t MIN_UDP_DATAGRAM_SIZE{ETHERNET_HEADER_SIZE + MIN_IP_V4_HEADER_SIZE +
                                                   UDP_HEADER_SIZE};

/// @brief TCP header size, in bytes
static constexpr std::size_t MIN_TCP_HEADER_SIZE{20U};

/// @brief Maximum length of a TCP payload, in bytes
static constexpr std::size_t TCP_PAYLOAD_MAX_LENGTH{DEFAULT_INTERFACE_MTU_SIZE -
                                                    MIN_IP_V4_HEADER_SIZE - MIN_TCP_HEADER_SIZE};

/// @brief Ethernet protocol type enum
enum class EthernetProtocolType : std::uint16_t
{
  IPV4 = 0x0800, ///< IPv4 protocol
  IPV6 = 0x86DD, ///< IPv6 protocol
  VLAN = 0x8100, ///< VLAN tag
};

/// @brief IP protocol type enum
enum class IpProtocolType : std::uint8_t
{
  TCP = 6,  ///< TCP protocol
  UDP = 17, ///< UDP protocol
};

/// @brief Number of octets contained in a MAC address.
static constexpr std::size_t MAC_ADDRESS_LENGTH{6U};

/// @brief Ethernet frame header
PACKED_DATA(struct EthernetHeader {
  std::array<std::uint8_t, MAC_ADDRESS_LENGTH> dest_mac_address; ///< destination MAC address
  std::array<std::uint8_t, MAC_ADDRESS_LENGTH> src_mac_address;  ///< source MAC address
  std::uint16_t ether_proto_type; ///< Ethernet protocol type; see EthernetProtocolType
});

/// @brief VLAN tag
PACKED_DATA(struct VlanTag {
  std::uint16_t priority_code_point : 3;     ///< Priority code point
  std::uint16_t drop_eligible_indicator : 1; ///< Drop eligible indicator
  std::uint16_t vlan_id : 12;                ///< VLAN identifier
  std::uint16_t ether_proto_type;            ///< Ethernet protocol type; see EthernetProtocolType
});

/// @brief IPv4 packet header
struct IpHeader
{
#ifdef __BYTE_ORDER
#if __BYTE_ORDER == __LITTLE_ENDIAN
  std::uint8_t header_len : 4; ///< IPv4 header length (in 4-byte words)
  std::uint8_t version : 4;    ///< IP version (always 4)
#elif __BYTE_ORDER == __BIG_ENDIAN
  std::uint8_t version : 4;    ///< IP version (always 4)
  std::uint8_t header_len : 4; ///< IPv4 header length (in 4-byte words)
#else
#error "Invalid byte order."
#endif
#else
#error "Byte order is not defined."
#endif
  std::uint8_t tos;                  ///< type of service
  std::uint16_t total_packet_len;    ///< the total packet length, includes header and data.
  std::uint16_t id;                  ///< identification
  std::uint16_t frag_offset;         ///< fragment offset
  std::uint8_t ttl;                  ///< time to live
  IpProtocolType ip_proto_type;      ///< IPv4 protocol type
  std::uint16_t checksum;            ///< checksum
  std::uint32_t source_address;      ///< source IPv4 address
  std::uint32_t destination_address; ///< destination IPv4 address
};

// Assert that these structs are the expected size.
static_assert(sizeof(EthernetHeader) == ETHERNET_HEADER_SIZE,
              "Ethernet header struct size does not match the Ethernet standard header size.");
static_assert(sizeof(VlanTag) == VLAN_TAG_SIZE,
              "VLAN tag struct size does not match the expected vlan tag size.");
static_assert(sizeof(IpHeader) == MIN_IP_V4_HEADER_SIZE,
              "IPv4 header struct size does not match the minimum IPv4 standard header size.");

} // namespace networking
} // namespace types
} // namespace common
} // namespace lum

#endif
