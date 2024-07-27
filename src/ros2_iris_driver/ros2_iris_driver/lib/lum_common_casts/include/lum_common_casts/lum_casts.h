// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_CASTS_LUM_CASTS_H
#define LUM_COMMON_CASTS_LUM_CASTS_H

#include <cstring>
#include <type_traits>

namespace lum {
namespace common {
namespace casts {

template <typename To, typename From>
inline auto lum_reinterpret_cast(From* from)
{
  return static_cast<typename std::conditional<std::is_const<From>{}, const To, To>::type>(
    static_cast<typename std::conditional<std::is_const<From>{}, const void*, void*>::type>(from));
}

/// @brief Used for SFINAE evaluation if T and U have the same size in bytes
template <class T, class U>
using EnabledIfAreSameSize = std::enable_if_t<sizeof(T) == sizeof(U)>;

/// @brief Used for SFINAE evaluation if T is default-constructible
template <class T>
using EnabledIfIsDefaultConstructible = std::enable_if_t<std::is_default_constructible<T>::value>;

/// @brief Used for SFINAE evaluation if T is trivially copyable
template <class T>
using EnabledIfIsTriviallyCopyable = std::enable_if_t<std::is_trivially_copyable<T>::value>;

/// @brief C++14 implementation of C++20's std::bit_cast
/// @details See the tests for usage.
/// @tparam To the type to convert into.
/// @tparam From the type to convert from.
template <class To,
          class From,
          class = EnabledIfAreSameSize<To, From>,
          class = EnabledIfIsDefaultConstructible<To>,
          class = EnabledIfIsTriviallyCopyable<From>,
          class = EnabledIfIsTriviallyCopyable<To>>
To lum_bit_cast(From from)
{
  To to{};
  std::memcpy(&to, &from, sizeof(From));
  return to;
}

} // namespace casts
} // namespace common
} // namespace lum

#endif // LUM_COMMON_CASTS_LUM_CASTS_H
