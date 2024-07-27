// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_TYPES_I_UDP_SOCKET_H
#define LUM_PLATFORM_NETWORKING_TYPES_I_UDP_SOCKET_H

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <lum_platform_networking_types/types.h>

namespace lum {
namespace platform {
namespace networking {

/// @brief UDP Socket Interface
/// @code
/// std::shared_ptr<lum::platform_networking::IUDPSocket> socket;
/// socket = lum::pdk::drivers::data_client::socket::createUDPSocket();
/// auto resp = socket->open();
/// if ( resp != lum::platform::networking::UDPSocketResponse::SUCCESS )
/// {
///   // ...
/// }
/// resp = socket->setReuseAddress( true );
/// if ( resp != lum::platform::networking::UDPSocketResponse::SUCCESS )
/// {
///   // ...
/// }
/// resp = socket->bind( 11000 );
/// if ( resp != lum::platform::networking::UDPSocketResponse::SUCCESS )
/// {
///   // ...
/// }
/// constexpr std::size_t buffer_size{256};
/// std::uint8_t buffer[buffer_size];
/// const auto bytes_read = socket->tryRead( buffer, buffer_size );
/// if ( bytes_read == 0 )
/// {
///   // ... no message received, may try again as needed
/// }
/// else
/// {
///   //  ... message received
/// }
/// socket->close();
/// @endcode
class IUDPSocket
{
public:
  IUDPSocket() = default;
  virtual ~IUDPSocket() = default;
  IUDPSocket(const IUDPSocket&) = delete;
  IUDPSocket(IUDPSocket&&) = delete;
  IUDPSocket& operator=(const IUDPSocket&) & = delete;
  IUDPSocket& operator=(IUDPSocket&&) & = delete;

  /// @brief set whether the socket can be reused
  /// @param [in] reuse whether the sockets bound address should be set for reuse or not
  /// @returns UDPSocketResponse enum for SUCCESS and errors
  virtual SocketResponse setReuseAddress(const bool reuse) = 0;

  /// @brief sets the timeout used during a tryRead() or tryReadFrom() operation
  /// @param [in] timeout duration to block on socket before returning with no message; a value of 0
  /// shall be used if the desired behavior is to not timeout.
  /// @returns SocketResponse enum for SUCCESS and errors
  virtual SocketResponse setSocketPollTimeout(const std::chrono::milliseconds& timeout) = 0;

  /// @brief set UDP socket to broadcast
  /// @returns SocketResponse enum for SUCCESS and errors
  virtual SocketResponse setSocketBroadcast() = 0;

  /// @brief Join multicast group on UDP socket
  /// @param [in] config the multicast configuration
  /// @return a SocketResponse indicating success or failure
  virtual SocketResponse joinMulticastGroup(const MulticastConfig& config) = 0;

  /// @brief close this socket
  virtual void close() = 0;

  /// @brief try to open the UDP socket
  /// @returns SocketResponse enum for SUCCESS and errors
  virtual SocketResponse open() = 0;

  /// @brief whether the socket is opened and in a listenable state
  /// @returns whether the socket has been opened
  virtual bool isOpen() const = 0;

  /// @brief attach an open socket to a specific local address, must be done before we can receive
  /// data
  /// @param [in] address local ip address to bind to for listening
  /// @param [in] port local port to receive messages on
  /// @returns SocketResponse enum for SUCCESS and errors
  virtual SocketResponse bind(const std::string& address, const std::uint16_t port) = 0;

  /// @brief attach an open socket to any local address, must be done before we can receive
  /// data
  /// @param [in] port local port to receive messages on
  /// @returns SocketResponse enum for SUCCESS and errors
  virtual SocketResponse bind(const std::uint16_t port) = 0;

  /// @brief whether the socket is bound to a port
  /// @returns whether the socket has been bound to a port
  virtual bool isBound() const = 0;

  /// @brief send a packet to the broadcast address
  /// @param [in] dest_port destination port for the message
  /// @param [in] payload pointer to the buffer containing the message bytes
  /// @param [in] payload_size number of bytes from the payload buffer to read and send
  /// @returns SocketResponse enum for SUCCESS and errors
  virtual SocketResponse
  broadcast(const std::uint16_t dest_port, const void* payload, const std::size_t payload_size) = 0;

  /// @brief send a UDP packet
  /// @param [in] dest_address the network address to target the message
  /// @param [in] dest_port destination port for the message
  /// @param [in] payload pointer to the buffer containing the message bytes
  /// @param [in] payload_size number of bytes from the payload buffer to read and send
  /// @returns SocketResponse enum for SUCCESS and errors
  virtual SocketResponse send(const std::string& dest_address,
                              const std::uint16_t dest_port,
                              const void* payload,
                              const std::size_t payload_size) = 0;

  /// @brief try to read from the socket, blocks forever, or until timeout expires
  /// @param [in] buffer raw pointer to the bytes to send
  /// @param [in] buffer_size number of bytes to read and send from the buffer
  /// @param [out] bytes_read the number of bytes read, will be zero for no data
  /// @returns SocketResponse enum for SUCCESS and errors
  virtual SocketResponse
  tryRead(void* buffer, const std::size_t buffer_size, std::size_t& bytes_read) = 0;

  /// @brief try to read from the socket, filtering by remote address, blocks forever or until
  /// timeout expires
  /// @param [in] buffer raw pointer to the bytes to send
  /// @param [in] buffer_size number of bytes to read and send from the buffer
  /// @param [out] bytes_read the number of bytes read, will be zero for no data
  /// @param [out] source_port the port the packet originated from (or zero if unknown)
  /// @returns UDPSocketResponse enum for SUCCESS and errors
  virtual SocketResponse tryReadFrom(void* buffer,
                                     const std::size_t buffer_size,
                                     std::size_t& bytes_read,
                                     std::uint16_t& source_port) = 0;

  /// @brief try to read from the socket, filtering by remote address, blocks forever or until
  /// timeout expires
  /// @param [in] buffer raw pointer to the bytes to send
  /// @param [in] buffer_size number of bytes to read and send from the buffer
  /// @param [out] bytes_read the number of bytes read, will be zero for no data
  /// @param [out] from_address address of packet
  /// @returns UDPSocketResponse enum for SUCCESS and errors
  virtual SocketResponse tryReadFrom(void* buffer,
                                     const std::size_t buffer_size,
                                     std::size_t& bytes_read,
                                     std::string& from_address) = 0;

  /// @brief try to read from the socket, filtering by remote address, blocks forever or until
  /// timeout expires
  /// @param [in] buffer raw pointer to the bytes to send
  /// @param [in] buffer_size number of bytes to read and send from the buffer
  /// @param [out] bytes_read the number of bytes read, will be zero for no data
  /// @param [out] from_address address of packet
  /// @param [out] source_port the port the packet originated from (or zero if unknown)
  /// @returns UDPSocketResponse enum for SUCCESS and errors
  virtual SocketResponse tryReadFrom(void* buffer,
                                     const std::size_t buffer_size,
                                     std::size_t& bytes_read,
                                     std::string& from_address,
                                     std::uint16_t& source_port) = 0;

  /// @brief try to read from the socket, filtering by remote address, blocks forever or until
  /// timeout expires (uses binary representation of remote address, IPv4 only)
  /// @param [in] buffer raw pointer to the bytes to send
  /// @param [in] buffer_size number of bytes to read and send from the buffer
  /// @param [out] bytes_read the number of bytes read, will be zero for no data
  /// @param [out] from_address address of packet
  /// @param [out] source_port the port the packet originated from (or zero if unknown)
  /// @returns UDPSocketResponse enum for SUCCESS and errors
  virtual SocketResponse tryReadFrom(void* buffer,
                                     const std::size_t buffer_size,
                                     std::size_t& bytes_read,
                                     std::uint32_t& from_address,
                                     std::uint16_t& source_port) = 0;
};

} // namespace networking
} // namespace platform
} // namespace lum

#endif
