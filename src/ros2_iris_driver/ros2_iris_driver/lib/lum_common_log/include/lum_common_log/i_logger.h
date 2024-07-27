// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_I_LOGGER_H
#define LUM_COMMON_LOG_I_LOGGER_H

#include <atomic>

#include <lum_common_log/types.h>

namespace lum {
namespace common {
namespace log {

/// @brief Base interface for a logger.
class ILogger
{
public:
  ILogger() = default;
  virtual ~ILogger() = default;

  ILogger(const ILogger&) = delete;
  ILogger(ILogger&&) = delete;
  ILogger& operator=(const ILogger&) & = delete;
  ILogger& operator=(ILogger&&) & = delete;

  /// @brief The method to write the message data record.
  /// @param[in] log_record The message data record. Join ownership if message content should be
  /// kept alive.
  virtual void sendLogMessage(LogRecord log_record) = 0;

  /// @brief Override this function to setup the message data record with logger-specific info.
  /// @note This method is only called if use_default_log_format_ == false.
  /// @param[in,out] log_record The message data record.
  virtual void initLogMessage(const LogRecord&) {}

  /// @brief Allow/forbid potential message coloring.
  /// @param[in] on If true try to colorize the output if supported by the logger.
  virtual void colorizeLogMessage(bool on) noexcept { colorize_ = on; }

  /// @brief Set the log level up-to and including which log messages should be discarded.
  /// @param[in] level The discard level to use.
  virtual void setDiscardLevel(LogLevel level) noexcept { discard_level_ = level; }

  /// @brief Query whether this logger has a custom message format.
  /// @return True if this logger is using the default log message format. If false, a custom format
  /// is produced by calling initLogMessage() on this logger.
  virtual bool isUsingDefaultLogFormat() const noexcept { return use_default_log_format_; }

protected:
  /// @brief Suppress logging of any message on and below this level.
  std::atomic<LogLevel> discard_level_{LogLevel::LEVEL_DEBUG};

  /// @brief If true, the logger may insert ANSI color sequences when writing the message string.
  std::atomic<bool> colorize_{true};

  /// @brief If false, call ThisLogger::initLogMessage() to setup the message output format.
  std::atomic<bool> use_default_log_format_{true};
};

} // namespace log
} // namespace common
} // namespace lum

#endif
