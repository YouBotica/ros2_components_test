// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_PLATFORM_NETWORKING_TYPES_H
#define LUM_PLATFORM_NETWORKING_TYPES_H

#include <cstdint>
#include <string>

#if defined(_WIN32)
#include <lum_platform_networking/windows/types.h>
#else
#include <lum_platform_networking/posix/types.h>
#endif

namespace lum {
namespace platform {

static constexpr std::uint32_t FLAG_TRUE{1U};
static constexpr std::uint32_t FLAG_FALSE{0U};

} // namespace platform
} // namespace lum

#endif
