// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_LOG_LOG_MESSAGE_H
#define LUM_COMMON_LOG_LOG_MESSAGE_H

#include <lum_common_log/i_logger.h>

namespace lum {
namespace common {
namespace log {

/// @brief Log message with data.
///
/// This class is instantiated upon every call to LUM_LOG_.... and operates
/// the logger passed in.
class LogMessage final
{
public:
  /// @brief Get text for the severity level.
  /// @param[in] level The log level
  /// @return Pointer to constant C-type string for the given level
  static const char* getLevelName(LogLevel level) noexcept;

  /// @brief Create a log message
  /// @param[in] path Source code file path causing this log message
  /// @param[in] line Source code line causing this log message
  /// @param[in] level Log severity level
  /// @param[in] output_logger Logger to stream the formatted message to
  LogMessage(const char* path, std::uint32_t line, LogLevel level, ILogger& output_logger);
  ~LogMessage() noexcept;

  LogMessage(const LogMessage&) = delete;
  LogMessage(LogMessage&&) = delete;
  LogMessage& operator=(const LogMessage&) & = delete;
  LogMessage& operator=(LogMessage&&) & = delete;

  /// @brief Get the stream to log to.
  /// @return Stream reference.
  std::ostream& getStream() const noexcept;

private:
  ILogger& logger_;
  LogRecord log_record_;
};

/// @brief Conversion function to create a LogEntry from internal logging data.
/// @param[in] log_record Handle to internal logging data
/// @return Public accessible data of a LogRecord
LogEntry getLogEntry(const LogRecord& log_record);

} // namespace log
} // namespace common
} // namespace lum

#endif
