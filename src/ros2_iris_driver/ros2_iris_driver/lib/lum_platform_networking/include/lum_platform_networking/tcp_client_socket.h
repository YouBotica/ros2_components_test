// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_TCP_CLIENT_SOCKET_H
#define LUM_PLATFORM_NETWORKING_TCP_CLIENT_SOCKET_H

#include <memory>

#include <lum_platform_networking_types/i_tcp_client_socket.h>

namespace lum {
namespace platform {
namespace networking {

/// @brief construct and return an implementation of ITCPClientSocket
/// @return a default ITCPClientSocket implementation
std::unique_ptr<ITCPClientSocket> makeTcpSocket();

} // namespace networking
} // namespace platform
} // namespace lum

#endif
