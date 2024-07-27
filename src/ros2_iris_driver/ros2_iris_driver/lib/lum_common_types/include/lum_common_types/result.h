// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_RESULT_H
#define LUM_COMMON_TYPES_RESULT_H

namespace lum {
namespace common {
namespace types {

/// @brief A combined return value with an error status
/// @details Akin to C++23's proposed std::expected (P0323) and Rust's Result type, this type is
/// also comparable to std::pair<T,bool> and std::optional<T> but adds semantic information as to
/// _why_ the operation failed, not just _if_ it failed.
/// @tparam T The substantive type of the return value (must be default constructible)
/// @tparam E The error information (typically an enumeration)
template <class T, typename E>
struct Result
{
  T value;
  E error;
};

} // namespace types
} // namespace common
} // namespace lum

#endif
