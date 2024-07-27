// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_SYSTEM_CPU_LUM_PLATFORM_SYSTEM_CPU_H
#define LUM_PLATFORM_SYSTEM_CPU_LUM_PLATFORM_SYSTEM_CPU_H

#include <cinttypes>
#include <set>
#include <thread>

namespace lum {
namespace platform {
namespace system {
namespace cpu {

/// @brief Sets the CPU affinity for the current process
/// @note This function assumes you're counting cpus from 1, not 0
/// @param cpu the CPU to bind to
/// @return whether the binding was successful
bool setCPUAffinity(std::uint32_t cpu);

/// @brief Sets the CPU affinity for the specified thread
/// @param t [in] Thread which needs to bind
/// @param cpu [in] the CPU to bind to
/// @note This function assumes you're counting cpus from 1, not 0
/// @return whether the binding was successful
bool setThreadCPUAffinity(std::thread& t, std::uint32_t cpu);

/// @brief Return the set of CPUs the thread is bound to
/// @param t [in] the thread
/// @note This function assumes you're counting cpus from 1, not 0
/// @return a set of bound CPUs
std::set<std::uint32_t> getBoundCPUs(std::thread& t);

/// @brief Return the number of physical CPU cores
/// @return the number of physical CPU cores
std::uint16_t getNumberOfCPUCores();

} // namespace cpu
} // namespace system
} // namespace platform
} // namespace lum

#endif // LUM_PLATFORM_SYSTEM_CPU_LUM_PLATFORM_SYSTEM_CPU_H
