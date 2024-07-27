// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.



#ifndef LUM_PLATFORM_SYSTEM_MEMORY_LUM_PLATFORM_SYSTEM_MEMORY_H
#define LUM_PLATFORM_SYSTEM_MEMORY_LUM_PLATFORM_SYSTEM_MEMORY_H

#include <cinttypes>
#include <set>

namespace lum {
namespace platform {
namespace system {
namespace memory {

/// @brief Return the memory capacity
/// @return the memory size
std::uint64_t getMemorySize();

} // namespace memory
} // namespace system
} // namespace platform
} // namespace lum

#endif // LUM_PLATFORM_SYSTEM_MEMORY_LUM_PLATFORM_SYSTEM_MEMORY_H
