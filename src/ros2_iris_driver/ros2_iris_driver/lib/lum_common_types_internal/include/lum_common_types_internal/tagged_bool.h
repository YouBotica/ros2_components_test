// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_INTERNAL_TAGGED_BOOL_H
#define LUM_COMMON_TYPES_INTERNAL_TAGGED_BOOL_H

namespace lum {
namespace common {
namespace types {
/// Enumeration to communicate at call site whether we're in test mode or not.
///
/// @code{.cpp}
///   // N.B. the keword `class` inside the template instatiation.
///   // This makes it so we don't have to declare these "tag" classes elsewhere.
///   using WithHardware    = TaggedBool<class WithHardwareTag>;
///   using WithDebugPrints = TaggedBool<class WithDebugPrintsTag>;
///
///   void process( const WithHardware with_hardware, const WithDebugPrints with_debug_prints )
///   {
///     if ( with_debug_prints ) // Implicit conversion to bool
///     {
///       std::cout << "Some info\n";
///     }
///     // ...
///     if ( !with_hardware )
///     {
///       simulate();
///     }
///   }
///
///   // At the call site, we're not stuck with `process(true, false);`, whose meaning is opaque.
///   // Instead, we name the values and cannot mix up their order accidentally:
///   process( WithHardware{true}, WithDebugPrints{false} );
///
/// @endcode
/// @note Adapted from https://akrzemi1.wordpress.com/2017/02/16/toggles-in-functions,
/// specifically making AUTOSAR friendly (removing friends) and applying our coding
/// standard namings to:
/// https://github.com/akrzemi1/explicit/blob/master/include/ak_toolkit/tagged_bool.hpp
///
/// @copyright Copyright (C) 2016 Andrzej Krzemienski.
///
/// Use, modification, and distribution is subject to the Boost Software
/// License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
/// http://www.boost.org/LICENSE_1_0.txt)
///
/// Adaptations (C) 2021 Luminar Technologies, Inc.
///
template <typename Tag>
class TaggedBool
{
public:
  constexpr explicit TaggedBool(bool v) : value_{v} {}

  constexpr explicit TaggedBool(int) = delete;
  constexpr explicit TaggedBool(double) = delete;
  constexpr explicit TaggedBool(void*) = delete;

  template <typename OtherTag>
  constexpr explicit TaggedBool(TaggedBool<OtherTag> b) : value_{bool(b)}
  {
  }

  constexpr explicit operator bool() const { return value_; }
  constexpr TaggedBool operator!() const { return TaggedBool{!value_}; }

  constexpr bool operator==(TaggedBool r) const { return value_ == r.value_; }
  constexpr bool operator!=(TaggedBool r) const { return value_ != r.value_; }

  constexpr bool operator==(bool b) const { return value_ == b; }
  constexpr bool operator!=(bool b) const { return value_ != b; }

private:
  bool value_;
};

template <class T>
constexpr bool operator==(bool b, TaggedBool<T> t)
{
  return t == b;
}

template <class T>
constexpr bool operator!=(bool b, TaggedBool<T> t)
{
  return t != b;
}

} // namespace types
} // namespace common
} // namespace lum
#endif
