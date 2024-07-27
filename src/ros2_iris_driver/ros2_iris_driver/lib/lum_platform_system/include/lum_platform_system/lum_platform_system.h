// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_SYSTEM_LUM_PLATFORM_SYSTEM_H
#define LUM_PLATFORM_SYSTEM_LUM_PLATFORM_SYSTEM_H

#include <ctime>
#include <string>
#include <vector>

namespace lum {
namespace platform {
namespace system {

/// @brief Gets system timezone offset
/// @param [in] Instance of time for which offset is required. Should be -1 for current time.
/// @return Timezone offset in string format
std::string getSystemTimeZoneOffset(time_t when);

/// @brief Helper function to get output of a terminal command
/// @param cmd_str [in] Command with flags to make a system call as vector of string
/// @return string of output from system call
std::string getStdoutFromCommand(std::vector<std::string> cmd_str);

} // namespace system
} // namespace platform
} // namespace lum

#endif // LUM_PLATFORM_SYSTEM_LUM_PLATFORM_SYSTEM_H
