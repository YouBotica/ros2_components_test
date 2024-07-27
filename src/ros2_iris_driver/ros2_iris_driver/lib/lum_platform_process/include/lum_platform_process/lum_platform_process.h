// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_PROCESS_LUM_PLATFORM_PROCESS_H
#define LUM_PLATFORM_PROCESS_LUM_PLATFORM_PROCESS_H

#include <string>

namespace lum {
namespace platform {
namespace process {

/// @brief Returns the current process ID as reported by the OS
/// @return the process ID
std::uint32_t getIdOfCurrentProcess();

/// @brief Returns the current thread ID as reported by the OS
/// @return the thread ID
std::uint32_t getIdOfCurrentThread();

/// @brief Try to name the current thread if supported by the OS
/// @param[in] name Name string
void setNameOfCurrentThread(const std::string& name);

/// @brief Try to get the name of the current thread if supported by the OS
/// @return Name or empty string if not supported
std::string getNameOfCurrentThread();

/// @brief Enables core dump for current system
bool enableCoreDump();

} // namespace process
} // namespace platform
} // namespace lum

#endif
