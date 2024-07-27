// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_TYPES_H
#define LUM_COMMON_LOG_TYPES_H

#include <chrono>
#include <memory>
#include <string>

namespace lum {
namespace common {
namespace log {

/// @brief Enum representing verbosity levels
enum class LogLevel : std::uint32_t
{
  /// Unassigned, no messages
  LEVEL_NONE = 0U,
  /// Execution trace messages
  LEVEL_TRACE = 1U,
  /// Debug messages
  LEVEL_DEBUG = 2U,
  /// Informational messages
  LEVEL_INFO = 4U,
  /// Warning messages
  LEVEL_WARN = 8U,
  /// Recoverable error messages
  LEVEL_ERROR = 16U,
  /// Fatal error messages
  LEVEL_FATAL = 32U
};

/// @brief Internal logging data
class LogData;
/// @brief Handle to internal logging data
using LogRecord = std::shared_ptr<LogData>;

/// @brief Log entry for public use
struct LogEntry final
{
  /// @brief Monotonic clock for time-stamping log messages
  using Clock = std::chrono::steady_clock;
  /// @brief Point in time for log
  using Time = std::chrono::time_point<Clock>;

  /// @brief File name
  std::string filename;
  /// @brief Message
  std::string message;
  /// @brief Time when log message was produced
  Time time_stamp;
  /// @brief Process ID
  std::uint32_t process_id;
  /// @brief Thread ID
  std::uint32_t thread_id;
  /// @brief Message severity level
  LogLevel log_level;
  /// @brief Source code line number
  std::uint16_t line;
};

} // namespace log
} // namespace common
} // namespace lum

#endif
