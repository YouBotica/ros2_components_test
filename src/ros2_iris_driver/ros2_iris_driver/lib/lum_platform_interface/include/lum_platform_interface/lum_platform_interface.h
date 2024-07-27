// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_LUM_PLATFORM_INTERFACE_H
#define LUM_PLATFORM_LUM_PLATFORM_INTERFACE_H

namespace lum {
namespace platform {

/// @brief   Initializes any platform specific services
/// @details Currently does nothing on Linux but on Windows it calls `WSAStartup` to initialize
/// networking.
///          Try not to call this twice in the same process.
bool initialize();

/// @brief   Cleans up any resources used by the platform layer
/// @details Currently does nothing on Linux but on Windows it calls `WSAShutdown` to shutdown
/// networking.
bool shutdown();

} // namespace platform
} // namespace lum

#endif
