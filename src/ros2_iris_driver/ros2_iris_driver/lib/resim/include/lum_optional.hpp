#ifndef LIBRESIM_LUM_OPTIONAL__H
#define LIBRESIM_LUM_OPTIONAL__H

#if defined(_WIN32)
// MSVC does not support experimental/optional for c++14; use a bootleg boost version instead
#include <boost_optional.hpp>
template <typename T> using lum_optional = std::experimental::optional<T>;
#else
// optional is not supported until cpp17, so use experimental until then
#include <experimental/optional>
template <typename T> using lum_optional = std::experimental::optional<T>;
#endif

#endif
