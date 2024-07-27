// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_UDP_SOCKET_H
#define LUM_PLATFORM_NETWORKING_UDP_SOCKET_H

#include <memory>

#include <lum_platform_networking_types/i_udp_socket.h>

namespace lum {
namespace platform {
namespace networking {

/// @brief creates instances of IUDPSocket
/// @return an instance of IUDPSocket
std::unique_ptr<IUDPSocket> makeUdpSocket();

/// @brief creates shared instances of IUDPSocket
/// @return a shared instance of IUDPSocket
std::shared_ptr<IUDPSocket> makeSharedUdpSocket();

} // namespace networking
} // namespace platform
} // namespace lum

#endif
