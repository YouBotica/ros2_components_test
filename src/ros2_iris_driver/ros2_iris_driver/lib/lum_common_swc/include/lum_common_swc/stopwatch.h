// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_SWC_STOPWATCH_H
#define LUM_COMMON_SWC_STOPWATCH_H

#include <cstdint>

#include <lum_common_types/common.h>

namespace lum {
namespace common {
namespace swc {

/// @brief A high resolution stopwatch.
///
/// This is a tool for obtaining time duration measurements using the highest resolution
/// monotonic timers available on the system. Timings near the resolution limit may be slightly less
/// accurate. The stopwatch needs to be reset() before first use.
///
/// @note The measured wall clock time is not tied to the system clock and hence does
/// not suffer from system clock adjustments.
///
/// @note The CPU time is always zero on systems which do not support per-thread CPU time
/// queries. If unsupported, the getResolution() function will report a resolution of zero for CPU
/// time.
class Stopwatch final
{
public:
  /// @brief Measured durations
  struct Duration final
  {
    /// Wall clock time of the measured durations of the stopwatch
    types::Time wall_clock_time{};
    /// CPU time of the measured durations of the stopwatch
    types::Time cpu_time{};
  };

  /// @brief Reset the stopwatch.
  void reset() noexcept;

  /// @brief Get duration elapsed in terms of Time type.
  /// @return Time durations since last reset().
  Duration elapsed() const noexcept;

  /// @brief Get the stopwatch resolution in terms of Time type.
  /// @return Minimum stopwatch tick as time duration. Zero if time measurement is not supported.
  static Duration getResolution() noexcept;

private:
  // Internal duration representation.
  // This is platform dependent data. Nothing can be assumed about the stored value.
  using Datum = std::int64_t;

  Datum last_reset_wall_clock_{};
  Datum last_reset_cpu_{};
};

} // namespace swc
} // namespace common
} // namespace lum

#endif
