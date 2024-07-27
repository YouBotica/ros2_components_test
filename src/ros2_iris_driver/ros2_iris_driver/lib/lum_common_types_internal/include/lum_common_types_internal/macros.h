// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_INTERNAL_MACROS_H
#define LUM_COMMON_TYPES_INTERNAL_MACROS_H

#if defined(_MSC_VER)
/// @def Use to tightly pack structs
#define PACKED_DATA(...) __pragma(pack(push, 1)) __VA_ARGS__ __pragma(pack(pop))
#else
/// @def Use to tightly pack structs
#define PACKED_DATA(...) __VA_ARGS__ __attribute__((__packed__))
#endif

// Define byte order and endianness macros
#if defined(__linux__)
#include <endian.h>
#elif defined(__QNX__)

#include <net/netbyte.h>

#if defined(__LITTLEENDIAN__)
#define __BYTE_ORDER __LITTLEENDIAN__
#define __LITTLE_ENDIAN __LITTLEENDIAN__

#elif defined(__BIGENDIAN__)
#define __BYTE_ORDER __BIGENDIAN__
#define __BIG_ENDIAN __BIGENDIAN__

#else
#error "Endianness not set on QNX!"
#endif

#elif defined(_WIN32)
#define __BYTE_ORDER __LITTLE_ENDIAN
#define __LITTLE_ENDIAN 1234
#define __BIG_ENDIAN 4321
#endif

#include <type_traits>

/// @def Template parameter restricted to numeric types
#define LUM_NUMERIC_TEMPLATE_TYPE                                                                  \
  template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type>

/// @def Template parameter restricted to floating point types
#define LUM_FLOATING_TEMPLATE_TYPE                                                                 \
  template <typename T,                                                                            \
            typename = typename std::enable_if<std::is_floating_point<T>::value, T>::type>

#endif
