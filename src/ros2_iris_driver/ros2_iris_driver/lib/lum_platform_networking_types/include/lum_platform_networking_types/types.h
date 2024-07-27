// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_TYPES_TYPES_H
#define LUM_PLATFORM_NETWORKING_TYPES_TYPES_H

#include <cstdint>
#include <string>

// AUTOSAR Rule A16-0-1 exception justification: These preprocessor names are globally defined
// on Windows and must be undefined so as not to override our symbols. We cannot disable their
// symbols in third-party code, so we must undo them here.
#if defined(_WIN32)
#undef ERROR_NOT_CONNECTED
#undef ERROR
#undef SUCCESS
#endif

namespace lum {
namespace platform {
namespace networking {

/// @brief possible responses from UDPSocket interactions
enum class SocketResponse : std::uint32_t
{
  SUCCESS,
  ERROR_NOT_OPEN,
  ERROR_ALREADY_OPEN,
  ERROR_NOT_BOUND,
  ERROR_ALREADY_BOUND,
  ERROR_SOCKET,
  ADDRESS_MISMATCH,
  TIMEOUT
};

/// @brief possible responses from TCPClientSocket interactions
enum class TCPClientSocketResponse : std::uint32_t
{
  SUCCESS,
  ERROR_NOT_OPEN,
  ERROR_ALREADY_OPEN,
  ERROR_NOT_CONNECTED,
  ERROR_ALREADY_CONNECTED,
  ERROR_SOCKET,
  TIMEOUT
};

enum class TCPServerSocketResponse : std::uint32_t
{
  SUCCESS,
  ERROR_NOT_OPEN,
  ERROR_ALREADY_OPEN,
  ERROR_NOT_BOUND,
  ERROR_ALREADY_BOUND,
  ERROR_NOT_LISTENING,
  ERROR_ALREADY_LISTENING,
  ERROR_NOT_RUNNING,
  ERROR_ALREADY_RUNNING,
  ERROR_NOT_CONNECTED,
  ERROR_ALREADY_CONNECTED,
  ERROR_SOCKET,
  TIMEOUT
};

/// @brief config used for joining a multicast group on a UDP socket
struct MulticastConfig
{
  std::string multicast_address{};
  std::string local_address{};
};

} // namespace networking
} // namespace platform
} // namespace lum

#endif
