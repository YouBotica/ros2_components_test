// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_UTILS_H
#define LUM_COMMON_TYPES_UTILS_H

#include <cstdint>
#include <limits>
#include <type_traits>

#ifndef NDEBUG
#include <lum_common_types/limit_check_on.h>
#else
#include <lum_common_types/limit_check_off.h>
#endif

namespace lum {
namespace common {
namespace types {
namespace utils {

/// @namespace lum::common::types::utils Utilities for POD type conversion

/// @brief Perform a static cast for integral types which is checked for range in debug builds
/// @tparam TO Type to cast to
/// @tparam FROM Current type of the value
/// @param [in] from A value as FROM type
/// @throw std::overflow_error
/// @return The value as TO type
template <typename TO,
          typename FROM,
          typename = typename std::enable_if<std::is_integral<TO>::value, TO>::type,
          typename = typename std::enable_if<std::is_integral<FROM>::value, FROM>::type>
inline TO numeric_cast(const FROM from)
{
  limit_check<TO, FROM>(from);
  return static_cast<TO>(from);
}

/// @private
namespace detail {

// Determines if a type is a char (signed or unsigned)
// T the type to evaluate
// clang-format off
/// @private
template <class T>
using IsCharType = std::integral_constant<bool, std::is_same<std::int8_t,  T>::value
                                             || std::is_same<std::uint8_t, T>::value>;
// clang-format on

// Removes const, volatile, and references from a type
// T the type to transform
/// @private
template <class T>
using StripExtras = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

// An undefined struct to be specialized depending on whether the type is an enum or
// integral (if neither, it will trigger an error because this type will not be instantiated)
// note: This works around issues in some older compilers related to std::underlying_type not
// being SFINAE friendly
// T the type to evaluate
// IsEnum Whether it is an enum type
// IsIntegral Whether it is an integral type
/// @private
template <class T,
          bool IsEnum = std::is_enum<T>::value,
          bool IsIntegral = std::is_integral<T>::value>
struct PrintableIntTypeImpl;

// A struct to define a nested type, specialized for enums
// T the type to evaluate
/// @private
template <class T>
struct PrintableIntTypeImpl<T, true, false>
{
  /// @brief The expose nested type
  using Type = typename std::conditional<IsCharType<typename std::underlying_type<T>::type>::value,
                                         std::int32_t,
                                         typename std::underlying_type<T>::type>::type;
};

// A struct to define a nested type, specialized for integral types.
// T the type to evaluate
/// @private
template <class T>
struct PrintableIntTypeImpl<T, false, true>
{
  /// @brief The expose nested type
  using Type = typename std::conditional<IsCharType<T>::value, std::int32_t, T>::type;
};

// Helper for toPrintableInt() to determine the type to use
// T The type to examine; will be stripped of const, volatile and references
/// @private
template <class T>
using PrintableIntType = typename PrintableIntTypeImpl<StripExtras<T>>::Type;

} // namespace detail

/// @brief Turn a value into a printable integer
/// @details Converts @a value to an integral type for printing, useful either to convert an enum to
/// a non-char-based integral type or to convert a native un/signed char to print as an integer
/// rather than a its ASCII value. Enums with un/signed char for their underlying type and also
/// un/signed chars themselves will become std::int32_t. Other enums will become their underlying
/// types. Other integral types will stay the same.
/// @tparam T The type to convert transform if needed.
/// @param[in] value The value to convert
/// @return The converted value
template <class T>
constexpr detail::PrintableIntType<T> toPrintableInt(const T value)
{
  return static_cast<detail::PrintableIntType<T>>(value);
}

/// @brief Turn enum value into integer
/// @details Converts enum @a value to its underlying integral representation
/// @tparam T An enum type
/// @param[in] value The value to convert
/// @return The converted value
template <typename T>
constexpr
  typename std::enable_if<std::is_enum<T>::value, typename std::underlying_type<T>::type>::type
  toUnderlyingValue(const T value) noexcept
{
  return static_cast<typename std::underlying_type<T>::type>(value);
}

} // namespace utils
} // namespace types
} // namespace common
} // namespace lum

#endif
