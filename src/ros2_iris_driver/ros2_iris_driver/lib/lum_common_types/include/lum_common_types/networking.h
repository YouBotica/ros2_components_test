// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_NETWORKING_H
#define LUM_COMMON_TYPES_NETWORKING_H

#include <array>
#include <cstdint>
#include <memory>

namespace lum {
namespace common {
namespace types {
namespace networking {

/// @namespace lum::common::types::networking Network interface data types

/// @brief Default MTU for network interfaces, in bytes
static constexpr std::size_t DEFAULT_INTERFACE_MTU_SIZE{1500U};

/// @brief Default UDP packet size, in bytes
static constexpr std::size_t DEFAULT_UDP_PACKET_SIZE{DEFAULT_INTERFACE_MTU_SIZE};

/// @brief UDP header size, in bytes
static constexpr std::size_t UDP_HEADER_SIZE{8U};

/// @brief UDP datagram header
struct UdpHeader
{
  std::uint16_t source_port;     ///< source port
  std::uint16_t dest_port;       ///< destination port
  std::uint16_t datagram_length; ///< datagram length, includes header and data.
  std::uint16_t checksum;        ///< checksum
};

/// @brief Structure for holding UDP packet data
/// @tparam N the size of the payload buffer (excludes the header)
template <std::uint16_t N>
class UdpPacketArray
{
public:
  /// @brief Get a pointer to the beginning of this buffer.
  /// @return Pointer to uint8_t
  std::uint8_t* data() noexcept { return buffer_.data(); }

  /// @brief Get a pointer to the beginning of this buffer.
  /// @return Pointer to uint8_t
  const std::uint8_t* data() const noexcept { return buffer_.data(); }

  /// @brief Get the available size of the buffer.
  /// @return The size of the buffer in bytes.
  std::size_t size() const noexcept { return buffer_.size(); }

  /// @brief Get length of valid data within this buffer.
  /// @return Valid length in bytes.
  std::size_t getPayloadLength() const noexcept
  {
    return static_cast<std::size_t>(payload_length_);
  }

  /// @brief Set the length of valid data within this buffer
  /// @post The retained payload length is the smaller of @a payload_length and the size of the
  /// internal buffer N
  /// @param[in] payload_length The number of bytes that are valid
  void setPayloadLength(const std::size_t payload_length) noexcept
  {
    if (payload_length < N)
    {
      payload_length_ = static_cast<std::uint16_t>(payload_length);
    }
    else
    {
      payload_length_ = N;
    }
  }

  /// @brief Overwrite the contents of this buffer with a given value.
  /// @param[in] val Value to set.
  void fill(const std::uint8_t val) noexcept(false) { buffer_.fill(val); }

  /// @brief Get value from buffer.
  /// @param[in] position Byte position in buffer.
  /// @return Writeable reference to value in buffer.
  std::uint8_t& operator[](const std::size_t position) noexcept(false)
  {
    return buffer_.at(position);
  }

  /// @brief Get value from buffer.
  /// @param[in] position Byte position in buffer.
  /// @return Readonly reference to value in buffer.
  const std::uint8_t& operator[](const std::size_t position) const noexcept(false)
  {
    return buffer_.at(position);
  }

  /// @brief Set the UDP header for this packet.
  /// @param[in] header The new header struct.
  void setHeader(const UdpHeader header) noexcept { header_ = header; }

  /// @brief Get the current UDP header for this packet.
  /// @return Reference to the current header struct.
  const UdpHeader& getHeader() const noexcept { return header_; }

protected:
  ~UdpPacketArray() noexcept = default;
  UdpPacketArray() noexcept = default;
  UdpPacketArray(const UdpPacketArray&) = default;
  UdpPacketArray(UdpPacketArray&&) noexcept = default;
  UdpPacketArray& operator=(const UdpPacketArray&) & = default;
  UdpPacketArray& operator=(UdpPacketArray&&) noexcept = default;

private:
  // the backing buffer for the payload
  std::array<std::uint8_t, N> buffer_{};
  // the header for this packet
  UdpHeader header_{};
  // length of valid data in the backing buffer
  std::uint16_t payload_length_{0U};
};

/// @brief UDP packet of default size.
class UdpPacket : public UdpPacketArray<DEFAULT_UDP_PACKET_SIZE>
{
};

/// @brief Reference to UDP packet of default size.
class UdpPacketRef
{
public:
  explicit UdpPacketRef(UdpPacket* const packet_ptr) noexcept
      : packet_ptr_{packet_ptr, [](UdpPacket* const p) noexcept { p->setPayloadLength(0U); }}
  {
  }

  /// @brief get a pointer to the beginning of the referenced UDP packet's buffer
  /// @return uint8_t pointer
  std::uint8_t* data() const noexcept { return packet_ptr_->data(); }

  /// @brief get the available size of the referenced UDP packet's buffer
  /// @return the size of the buffer in bytes
  std::size_t size() const noexcept { return packet_ptr_->size(); }

  /// @brief get length of valid data within the referenced UDP packet's buffer
  /// @return Valid length in bytes.
  std::size_t getPayloadLength() const noexcept { return packet_ptr_->getPayloadLength(); }

  /// @brief set the length of valid data within the referenced UDP packet's buffer
  /// @param payload_length the number of bytes that are valid, capped to the size of this buffer
  void setPayloadLength(const std::size_t payload_length) const noexcept
  {
    packet_ptr_->setPayloadLength(payload_length);
  }

  /// @brief overwrite the contents of the referenced UDP packet's buffer with a given value
  /// @param[in] val Value to set.
  void fill(const std::uint8_t val) const noexcept { packet_ptr_->fill(val); }

  /// @brief Get value from referenced buffer.
  /// @param[in] position Byte position in referenced buffer.
  /// @return Reference to value in buffer.
  std::uint8_t& operator[](const std::size_t position) noexcept(false)
  {
    return packet_ptr_->operator[](position);
  }

  /// @brief Get value from referenced buffer.
  /// @param[in] position Byte position in referenced buffer.
  /// @return Reference to value in buffer.
  const std::uint8_t& operator[](const std::size_t position) const noexcept(false)
  {
    return packet_ptr_->operator[](position);
  }

  /// @brief set the header for referenced UDP packet
  /// @param [in] header the new header struct
  void setHeader(const UdpHeader header) const noexcept { packet_ptr_->setHeader(header); }

  /// @brief get a const ref to the referenced UDP packet's header
  /// @return the current header struct
  const UdpHeader& getHeader() const noexcept { return packet_ptr_->getHeader(); }

private:
  const std::shared_ptr<UdpPacket> packet_ptr_;
};

/// @brief Type for port numbers
/// @note While all values are valid in principle, certain ports may be in use on any particular
/// machine
using PortType = std::uint16_t;

/// @brief Special port number signaling an invalid port.
static constexpr PortType INVALID_PORT{0U};

/// @brief The port used by the Telnet application protocol.
static constexpr PortType DEFAULT_TELNET_PORT{23U};

/// @brief Loopback Ethernet interface address as C-style string.
static constexpr const char* LOCALHOST_IP{"127.0.0.1"};

} // namespace networking
} // namespace types
} // namespace common
} // namespace lum

#endif
