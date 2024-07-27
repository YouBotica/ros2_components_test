// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_I_CLOCK_H
#define LUM_COMMON_SWC_I_CLOCK_H

#include <lum_common_types/common.h>

namespace lum {
namespace common {
namespace swc {

/// @brief Clock abstraction interface
class IClock
{
public:
  IClock() = default;
  virtual ~IClock() = default;

  IClock(const IClock&) = delete;
  IClock& operator=(const IClock&) & = delete;
  IClock(IClock&&) = delete;
  IClock& operator=(IClock&&) & = delete;

  /// @brief Get the current point in time.
  /// The time origin ("epoch") depends on the clock implementation.
  /// @return The time duration elapsed since the time origin of the clock.
  virtual types::Time now() const noexcept = 0;

  /// @brief Block the calling thread at least for @a duration.
  /// @note This might perform busy waiting especially for very short durations.
  /// @note The function may block longer than @a duration due to scheduling or resource contention
  /// delays.
  /// @param[in] duration Time duration to sleep.
  virtual void sleepFor(types::Time duration) = 0;

  /// @brief Query whether the clock is monotonically increasing.
  /// @return True if points in time never decrease, false otherwise.
  virtual bool isSteady() const noexcept = 0;

  /// @brief Get the clock resolution in terms of internal Time type.
  /// @return Minimum clock tick as time duration.
  virtual types::Time getResolution() const noexcept = 0;
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
