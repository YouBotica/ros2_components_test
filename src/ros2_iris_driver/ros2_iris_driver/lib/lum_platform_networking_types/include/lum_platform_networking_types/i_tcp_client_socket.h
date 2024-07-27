// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_TYPES_I_TCP_CLIENT_SOCKET_H
#define LUM_PLATFORM_NETWORKING_TYPES_I_TCP_CLIENT_SOCKET_H

#include <chrono>
#include <cstdint>
#include <string>

#include <lum_platform_networking_types/types.h>

namespace lum {
namespace platform {
namespace networking {

/// @brief TCP socket interface
/// @code
/// auto socket = lum::pdk::drivers::data_client::socket::createTCPSocket();
/// auto resp = socket->open();
/// if ( resp != lum::platform::networking::TCPClientSocketResponse::SUCCESS )
/// {
///   // ...
/// }
/// resp = socket->connect( "10.42.0.253", 50000 );
/// if ( resp != lum::platform::networking::TCPClientSocketResponse::SUCCESS )
/// {
///   // ...
/// }
///
/// constexpr std::size_t payload_size{256};
/// std::uint8_t payload[payload_size];
///
/// // fill payload ...
///
/// resp = socket->send( payload, payload_size );
/// if ( resp != lum::platform::networking::TCPClientSocketResponse::SUCCESS )
/// {
///   // ...
/// }
/// socket->close();
/// @endcode
class ITCPClientSocket
{
public:
  virtual ~ITCPClientSocket() = default;

  // Boilerplate to disable copy/move for a class used as polymorphic base
  // deleting the copy-constructor removes the implicit constructor, reinstate
  ITCPClientSocket() = default;
  ITCPClientSocket(const ITCPClientSocket&) = delete;
  ITCPClientSocket& operator=(const ITCPClientSocket&) & = delete;
  ITCPClientSocket(ITCPClientSocket&&) = delete;
  ITCPClientSocket& operator=(ITCPClientSocket&&) & = delete;
  // End Boilerplate

  /// @brief set "no delay" option on TCP client socket; turning on the "no delay" option will
  /// disable Nagle's algorithm
  /// @param [in] no_delay whether to turn on "no delay" option or not
  /// @returns TCPClientSocketResponse enum for SUCCESS and errors
  virtual TCPClientSocketResponse setNoDelayOption(const bool no_delay) = 0;

  /// @brief sets the timeout used during a tryRead() operation
  /// @param [in] timeout duration to block on socket before returning with no message; a value of 0
  /// shall be used if the desired behavior is to not timeout.
  /// @returns TCPClientSocketResponse enum for SUCCESS and errors
  virtual TCPClientSocketResponse
  setSocketPollTimeout(const std::chrono::milliseconds& timeout) = 0;

  /// @brief close this socket
  /// @returns TCPClientSocketResponse enum for SUCCESS and errors
  virtual TCPClientSocketResponse close() = 0;

  /// @brief try to open the TCP client socket
  /// @returns TCPClientSocketResponse enum for SUCCESS and errors
  virtual TCPClientSocketResponse open() = 0;

  /// @brief whether the socket is opened and in a listenable state
  /// @returns whether the socket has been opened
  virtual bool isOpen() const = 0;

  /// @brief attempts to establish a socket connection to a remote endpoint, timeout after 5
  /// seconds
  /// @param [in] address remote endpoint ip address
  /// @param [in] port remote endpoint port number
  /// @param [in] timeout optional connect timeout in seconds, default 5
  /// @returns TCPClientSocketResponse enum for SUCCESS and errors
  virtual TCPClientSocketResponse connect(const std::string& address,
                                          const std::uint16_t port,
                                          const std::uint16_t timeout = 5) = 0;

  /// @brief whether the socket is connected to a remote endpoint
  /// @returns whether the socket has been bound to a remote endpoint
  virtual bool isConnected() const = 0;

  /// @brief send a packet
  /// @param [in] payload pointer to the buffer containing the message bytes
  /// @param [in] payload_size number of bytes from the payload buffer to read and send
  /// @returns TCPClientSocketResponse enum for SUCCESS and errors
  virtual TCPClientSocketResponse send(const void* payload, const std::size_t payload_size) = 0;

  /// @brief try to read from the socket, blocks forever, or until timeout expires
  /// @param [in] buffer raw pointer to the bytes to send
  /// @param [in] buffer_size number of bytes to read and send from the buffer
  /// @param [out] bytes_read the number of bytes read, will be zero for no data
  /// @returns TCPClientSocketResponse enum for SUCCESS and errors
  virtual TCPClientSocketResponse
  tryRead(void* buffer, const std::size_t buffer_size, std::size_t& bytes_read) = 0;
};

} // namespace networking
} // namespace platform
} // namespace lum

#endif
