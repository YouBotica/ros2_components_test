// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_LIMIT_CHECK_ON_H
#define LUM_COMMON_TYPES_LIMIT_CHECK_ON_H

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace lum {
namespace common {
namespace types {
namespace utils {

/// @brief Perform a range check for narrowing integral conversion when the source type is unsigned
/// and the target type is signed.
/// @tparam TO Target type
/// @tparam FROM Source type
/// @param [in] from A value as FROM type
/// @throw std::overflow_error if narrowing the value would not fit into target type
template <typename TO, typename FROM>
typename std::enable_if<std::is_integral<TO>::value && std::is_signed<TO>::value &&
                        std::is_integral<FROM>::value && std::is_unsigned<FROM>::value>::type
limit_check(const FROM from) noexcept(false)
{
  // Check if the provided value is larger than the target max value
  if (from > static_cast<typename std::make_unsigned_t<TO>>(std::numeric_limits<TO>::max()))
  {
    throw std::overflow_error("numeric overflow detected when narrowing a type");
  }
}

/// @brief Perform a range check for narrowing integral conversion when the source type is signed
/// and the target type is unsigned.
/// @tparam TO Target type
/// @tparam FROM Source type
/// @param [in] from A value as FROM type
/// @throw std::overflow_error if narrowing the value would not fit into target type
template <typename TO, typename FROM>
typename std::enable_if<std::is_integral<TO>::value && std::is_unsigned<TO>::value &&
                        std::is_integral<FROM>::value && std::is_signed<FROM>::value>::type
limit_check(const FROM from) noexcept(false)
{
  // Check if the provided value is larger than the target max value or the provided value is less
  // than zero.
  if (static_cast<typename std::make_unsigned_t<FROM>>(from) > std::numeric_limits<TO>::max() ||
      from < 0)
  {
    throw std::overflow_error("numeric overflow detected when narrowing a type");
  }
}

/// @brief Perform a range check for narrowing integral conversion when the source and target types
/// are both signed or both unsigned
/// @tparam TO Target type
/// @tparam FROM Source type
/// @param [in] from A value as FROM type
/// @throw std::overflow_error if narrowing the value would not fit into target type
template <typename TO, typename FROM>
typename std::enable_if<std::is_integral<TO>::value && std::is_integral<FROM>::value &&
                        std::is_signed<TO>::value == std::is_signed<FROM>::value>::type
limit_check(const FROM from) noexcept(false)
{
  // Check if the provided value is larger than the target max value or smaller than the target min
  // value.
  if (from > std::numeric_limits<TO>::max() || from < std::numeric_limits<TO>::min())
  {
    throw std::overflow_error("numeric overflow detected when narrowing a type");
  }
}

} // namespace utils
} // namespace types
} // namespace common
} // namespace lum

#endif
