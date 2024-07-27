// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_LIMIT_CHECK_OFF_H
#define LUM_COMMON_TYPES_LIMIT_CHECK_OFF_H

#include <type_traits>

namespace lum {
namespace common {
namespace types {
namespace utils {

/// @brief Null operation for narrowing integral conversion
/// @tparam TO Target type
/// @tparam FROM Source type
/// @param [in] from A value as FROM type
template <typename TO,
          typename FROM,
          typename = typename std::enable_if<std::is_integral<TO>::value, TO>::type,
          typename = typename std::enable_if<std::is_integral<FROM>::value, FROM>::type>
inline void limit_check(const FROM from) noexcept
{
  (void)from;
}

} // namespace utils
} // namespace types
} // namespace common
} // namespace lum

#endif
