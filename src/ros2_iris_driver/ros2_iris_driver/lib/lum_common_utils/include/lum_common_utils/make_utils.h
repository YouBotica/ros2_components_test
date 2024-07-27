// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_UTILS_MAKE_UTILS_H
#define LUM_COMMON_UTILS_MAKE_UTILS_H

#include <array>
#include <type_traits>
#include <utility>

namespace lum {
namespace common {
namespace utils {

// Note: Implementation adapted from https://en.cppreference.com/w/cpp/container/array/to_array

// Internal details
namespace detail {

template <class T, std::size_t N, std::size_t... I>
constexpr std::array<std::remove_cv_t<T>, N> toArrayImpl(T (&a)[N], std::index_sequence<I...>)
{
  return {{a[I]...}};
}

template <class T, std::size_t N, std::size_t... I>
constexpr std::array<std::remove_cv_t<T>, N> toArrayImpl(T(&&a)[N], std::index_sequence<I...>)
{
  return {{std::move(a[I])...}};
}

template <class T, class U, std::size_t N, std::size_t... I>
constexpr std::array<std::remove_cv_t<T>, N> toArrayImpl(U (&a)[N], std::index_sequence<I...>)
{
  return {{T(a[I])...}};
}

template <class T, class U, std::size_t N, std::size_t... I>
constexpr std::array<std::remove_cv_t<T>, N> toArrayConvertImpl(U(&&a)[N],
                                                                std::index_sequence<I...>)
{
  return {{T(std::move(a[I]))...}};
}
} // namespace detail

/// @brief Make a std::array from the raw array of values.
/// @tparam T The type of an object to put in the array (usually deduced).
/// @tparam N Size of the array (usually deduced).
/// @return a std::array<T, N>, initialized with the items given
/// @note Borrowed from https://en.cppreference.com/w/cpp/container/array/to_array
///       Obsolete when we can use C++20's std::to_array() or deduction guides.
/// @code{.cpp}
/// const auto my_array = toArray({1, 2, 3, 4, 5}); // Deduces std::array<int,5>
/// auto pairs = toArray<std::pair<int, float>>( { { 3, .0f }, { 4, .1f }, { 4, .1e23f } } );
/// @endcode
template <class T, std::size_t N>
constexpr std::array<std::remove_cv_t<T>, N> toArray(T (&a)[N])
{
  return detail::toArrayImpl(a, std::make_index_sequence<N>{});
}

/// @brief Make a std::array from the rvalue raw array of values.
/// @tparam T The type of an object to put in the array (usually deduced).
/// @tparam N Size of the array (usually deduced).
/// @return a std::array<T, N>, initialized with the items given
/// @note Borrowed from https://en.cppreference.com/w/cpp/container/array/to_array
///       Obsolete when we can use C++20's std::to_array() or deduction guides.
/// @code{.cpp}
/// const auto my_array = toArray({1, 2, 3, 4, 5}); // Deduces std::array<int,5>
/// auto pairs = toArray<std::pair<int, float>>( { { 3, .0f }, { 4, .1f }, { 4, .1e23f } } );
/// @endcode
template <class T, std::size_t N>
constexpr std::array<std::remove_cv_t<T>, N> toArray(T(&&a)[N])
{
  return detail::toArrayImpl(std::move(a), std::make_index_sequence<N>{});
}

/// @brief Make a std::array from the raw array of values, converting each to T
/// @note Enabled only when T != U
/// @tparam T The type of the objects in the returned array.
/// @tparam U The type of the objects that are converted and added to the array; must satisfy
/// conversion T(const U&) or T(U&&).
/// @tparam N Size of the array (usually deduced).
/// @return a std::array<T, N>, initialized with the items given
/// @code{.cpp}
/// // Explicit argument needed, otherwise deduces std::array<int,5>
/// const auto my_array = toArray<std::uint8_t>({1, 2, 3, 4, 5});
/// @endcode
template <class T, class U, std::size_t N>
constexpr
  typename std::enable_if<!std::is_same<T, std::remove_cv_t<U>>::value, std::array<T, N>>::type
    toArray(U (&a)[N])
{
  return detail::toArrayConvertImpl<T>(a, std::make_index_sequence<N>{});
}

/// @brief Make a std::array from the rvalue raw array of values, converting each to the given
/// template param T
/// @note Enabled only when T != U
/// @tparam T The type of the objects in the returned array.
/// @tparam U The type of the objects that are converted and added to the array; must satisfy
/// conversion T(const U&) or T(U&&).
/// @tparam N Size of the array (usually deduced).
/// @return a std::array<T, N>, initialized with the items given
/// @code{.cpp}
/// // Explicit argument needed, otherwise deduces std::array<int,5>
/// const auto my_array = toArray<std::uint8_t>({1, 2, 3, 4, 5});
/// @endcode
template <class T, class U, std::size_t N>
constexpr
  typename std::enable_if<!std::is_same<T, std::remove_cv_t<U>>::value, std::array<T, N>>::type
    toArray(U(&&a)[N])
{
  return detail::toArrayConvertImpl<T>(std::move(a), std::make_index_sequence<N>{});
}

} // namespace utils
} // namespace common
} // namespace lum

#endif // LUM_COMMON_UTILS_MAKE_UTILS_H
