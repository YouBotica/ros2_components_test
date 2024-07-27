// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_SYSTEM_CLOCK_H
#define LUM_COMMON_SWC_SYSTEM_CLOCK_H

#include <lum_common_swc/i_clock.h>

namespace lum {
namespace common {
namespace swc {

/// @brief A system clock implementation
class SystemClock final : public IClock
{
public:
  /// @brief Get the current point in time.
  /// On most platforms, the time origin is the UNIX epoch, January 1, 1970
  /// @return The time duration elapsed since the time origin.
  types::Time now() const noexcept final;

  /// @copydoc IClock::sleepFor
  void sleepFor(types::Time duration) final;

  /// @copydoc IClock::isSteady
  bool isSteady() const noexcept final;

  /// @copydoc IClock::getResolution
  types::Time getResolution() const noexcept final;
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
