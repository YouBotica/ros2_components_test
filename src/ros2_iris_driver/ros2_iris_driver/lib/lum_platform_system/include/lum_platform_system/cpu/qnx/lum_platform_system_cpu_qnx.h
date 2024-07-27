// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_SYSTEM_CPU_QNX_LUM_PLATFORM_SYSTEM_CPU_QNX_H
#define LUM_PLATFORM_SYSTEM_CPU_QNX_LUM_PLATFORM_SYSTEM_CPU_QNX_H

#include <cinttypes>
#include <memory>
#include <thread>

#include <sched.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>

namespace lum {
namespace platform {
namespace system {
namespace cpu {
namespace qnx {

/// @brief Returns a shared ptr wrapping the thread's run mask
/// @param t [in] the thread
/// @return a possibly null unique pointer to the run mask
std::unique_ptr<std::uint32_t[]> fetchRunMask(std::thread& t);

} // namespace qnx
} // namespace cpu
} // namespace system
} // namespace platform
} // namespace lum

#endif // LUM_PLATFORM_SYSTEM_CPU_LUM_PLATFORM_SYSTEM_CPU_QNX_H
