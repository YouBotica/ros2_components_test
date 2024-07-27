// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_MATH_FLOATING_POINT_H
#define LUM_COMMON_MATH_FLOATING_POINT_H

#include <cmath>
#include <limits>
#include <type_traits>

namespace lum {
namespace common {
namespace math {

/// A "relative epsilon" comparison for equality which finds the difference between the two numbers
/// @a a and @a b compared to the machine epsilon of the type @c F when multiplied by the larger
/// magnitude of @a or @a b.
///
/// @details In order to get consistent results one should always compare the difference to the
/// larger of the two numbers. In other words, to compare @c a and @c b calculate
///   <tt>diff = abs(a-b)</tt>
/// If @c diff is smaller than <tt>epsilon * max(abs(a),abs(b))</tt> then @c a and @c b can be
/// considered equal.
///
/// For numbers larger than 2.0 the gap between floats grows larger, and if we compare floats using
/// FLT_EPSILON then we are just doing a more expensive and less obvious equality check. That is,
/// if two floats greater than 2.0 are not the same then their difference is guaranteed to be
/// greater than FLT_EPSILON. For floats above 16,777,216 the appropriate epsilon to use is greater
/// than one, and a comparison using FLT_EPSILON is, as it were, pointless.
///
/// @note This is implementation and description are adapted from _The Art of Computer Programming_
/// by Donald Knuth and
/// https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/ with
/// special cases from https://floating-point-gui.de/errors/comparison/
template <class F, typename CONSTRAINT = std::enable_if<std::is_floating_point<F>::value>>
bool areApproximatelyEqual(const F a, const F b) noexcept
{
  // Special cases: Bit patterns match exactly (good for zeros and if we get lucky with
  // floating-point numbers)
  if (a == b)
  {
    return true;
  }
  // Equality of infinities is caught above, so if only one of them is infinite, it is a guaranteed
  // mismatch.
  else if (std::isinf(a) || std::isinf(b) || std::isnan(a) || std::isnan(b))
  {
    return false;
  }

  // If a or b is zero or both are extremely close to it, relative error is less meaningful
  constexpr auto EPSILON = std::numeric_limits<F>::epsilon();
  constexpr auto DENORM_MIN = std::numeric_limits<F>::denorm_min();
  const auto abs_a = std::abs(a);
  const auto abs_b = std::abs(b);
  const auto diff = std::abs(a - b);

  if (a == 0 || b == 0 || (abs_a + abs_b < DENORM_MIN))
  {
    return diff < (EPSILON * DENORM_MIN);
  }

  // Return the diff compared to a scaled epsilon for relative error
  const auto larger = abs_b > abs_a ? abs_b : abs_a;
  return diff <= larger * EPSILON;
}

/// Simplified, special case of `areApproximatelyEqual` for comparing with zero
/// @param [in] a value to be checked
/// @return true if parameter is approximately equal to zero
template <class F, typename CONSTRAINT = std::enable_if<std::is_floating_point<F>::value>>
bool isApproximatelyZero(const F a) noexcept
{
  if (std::isinf(a) || std::isnan(a))
  {
    return false;
  }

  constexpr auto DENORM_MIN = std::numeric_limits<F>::denorm_min();
  constexpr auto MARGIN = static_cast<F>(1e2);
  const auto abs_a = std::abs(a);

  return abs_a < (DENORM_MIN * MARGIN);
}

} // namespace math
} // namespace common
} // namespace lum

#endif
