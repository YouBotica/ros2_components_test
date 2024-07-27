// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_UUID_H
#define LUM_COMMON_UTILS_UUID_H

#include <string>

namespace lum {
namespace common {
namespace utils {

/// @brief Generates a uuid
/// @return uuid as a string
std::string generateUUIDV4();

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_UUID_H
