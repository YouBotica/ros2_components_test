// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_POSIX_TYPES_H
#define LUM_PLATFORM_NETWORKING_POSIX_TYPES_H

#include <cstdint>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace lum {
namespace platform {
namespace networking {

using PlatformSocket = std::int32_t;
constexpr PlatformSocket DEFAULT_PLATFORM_SOCKET = -1;

} // namespace networking
} // namespace platform
} // namespace lum

#endif
